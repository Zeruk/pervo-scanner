/* HTTP Restful API Server

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <fcntl.h>
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include <dirent.h>
#include <stdio.h>

#include "uln2003.h"
#include "sdCard.h"
#include "ydlidar.h"

static const char *REST_TAG = "REST";
#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            ESP_LOGE(REST_TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context
{
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html"))
    {
        type = "text/html";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".js"))
    {
        type = "application/javascript";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".css"))
    {
        type = "text/css";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".png"))
    {
        type = "image/png";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".ico"))
    {
        type = "image/x-icon";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".svg"))
    {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* Send HTTP response with the contents of the requested file */
static esp_err_t rest_common_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/')
    {
        strlcat(filepath, "/index.html", sizeof(filepath));
    }
    else
    {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    ESP_LOGI(REST_TAG, "Filepath is %s", filepath);
    FILE *fp;
    fp = fopen(filepath, "r");
    if (fp == NULL)
    {
        ESP_LOGE(REST_TAG, "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    int16_t read_bytes;
    do
    {
        /* Read file in chunks into the scratch buffer */
        read_bytes = fread(chunk, sizeof(char), SCRATCH_BUFSIZE, fp);
        ESP_LOGI(REST_TAG, "Readed %d, buffer: %d", read_bytes, SCRATCH_BUFSIZE);
        if (read_bytes == -1)
        {
            ESP_LOGE(REST_TAG, "Failed to read file : %s", filepath);
        }
        else if (read_bytes > 0)
        {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK)
            {
                fclose(fp);
                ESP_LOGE(REST_TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (!feof(fp));
    /* Close file after sending complete */
    fclose(fp);
    ESP_LOGI(REST_TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Simple handler for light brightness control */
static esp_err_t file_control_rename_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    char *before = cJSON_GetObjectItem(root, "before")->valuestring;
    char *after = cJSON_GetObjectItem(root, "after")->valuestring;
    ESP_LOGI(REST_TAG, "before: %s, after: %s", before, after);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
    return ESP_OK;
}

static esp_err_t file_control_list_handler(httpd_req_t *req)
{
    if (SDCard.state < 1)
        SDCard.init();
    if (SDCard.state < 1)
    {
        esp_err_thttpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"error\": 2,\"desc\":\"Can't access SD card\"}");
        return ESP_OK;
    }

    cJSON *root = cJSON_CreateObject();
    // cJSON_AddStringToObject(root, "version", IDF_VER);

    cJSON *files = cJSON_AddArrayToObject(root, "files");

    DIR *d;
    struct dirent *dir;
    d = opendir("/sdcard");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            if (dir->d_type == DT_REG)
            {
                cJSON_AddItemToArray(files, cJSON_CreateString(dir->d_name));
                printf("%s\n", dir->d_name);
            }
        }
        closedir(d);
    }

    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Simple handler for getting system handler */
static esp_err_t system_info_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cJSON_AddStringToObject(root, "version", IDF_VER);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Simple handler for getting temperature data */
static esp_err_t temperature_data_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "raw", esp_random() % 20);
    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t scan_control_handler(httpd_req_t *req)
{
    // TODO: add SDCARD newfile counter
    // [backward|forward|stop]
    ESP_LOGI(TAG, "Initialize YdlidarController");
    YdlidarController.init();
    if (NULL != strstr(req->uri, "start"))
    {
        if (SDCard.state < 1)
            SDCard.init();
        if (SDCard.state < 1)
        {
            esp_err_thttpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"error\": 2,\"desc\":\"Can't access SD card\"}");
            return ESP_OK;
        }
        SDCard.newFile("/sdcard/scan1.xyz");
        if (SDCard.state < 2)
        {
            esp_err_thttpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"error\": 2,\"desc\":\"Can't create file\"}");
            return ESP_OK;
        }
        // TODO: Add capacitor near YDLidar
        YdlidarController.start();
        // TODO: Add rotation position lookup, disable YDLidar after
    }
    else if (NULL != strstr(req->uri, "stop"))
    {
        YdlidarController.stop();
        SDCard.closeFile();
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }
    else
    { //status
        httpd_resp_set_type(req, "application/json");
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "progress", StepperControl.progress);
        const char *status = cJSON_Print(root);
        httpd_resp_sendstr(req, status);
        free((void *)status);
        cJSON_Delete(root);
        return ESP_OK;
    }
}

static esp_err_t rotate_control_handler(httpd_req_t *req)
{
    // [backward|forward|stop]
    ESP_LOGI(TAG, "Initialize StepperControl");
    StepperControl.init();
    if (NULL != strstr(req->uri, "backward"))
    {
        StepperControl.direction = -1;
    }
    else if (NULL != strstr(req->uri, "forward"))
    {
        StepperControl.direction = 1;
    }
    else
    {
        StepperControl.direction = 0;
        // TODO: disable Stepper Motor
    }
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

esp_err_t start_rest_server(const char *base_path)
{
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "No memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(REST_TAG, "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err_start);

    /////////////////////////// REGISTERING HANDLERS ///////////////////////////

    /* URI handler for fetching system info */
    httpd_uri_t system_info_get_uri = {
        .uri = "/api/v1/system/info",
        .method = HTTP_GET,
        .handler = system_info_get_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &system_info_get_uri);

    /* URI handler for fetching temperature data */
    httpd_uri_t temperature_data_get_uri = {
        .uri = "/api/v1/temp/raw",
        .method = HTTP_GET,
        .handler = temperature_data_get_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &temperature_data_get_uri);

    // /api/v1/scan/[start|status|stop]
    // /api/v1/rotate/[backward|forward|stop]
    // /api/v1/files/list // TODO: if not connected to SDCARD, show err message

    /* /api/v1/scan/stop */
    httpd_uri_t scan_stop_post_uri = {
        .uri = "/api/v1/scan/*",
        .method = HTTP_POST,
        .handler = scan_control_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &scan_stop_post_uri);

    /* /api/v1/rotate/[backward|forward|stop] */
    httpd_uri_t rotate_control_post_uri = {
        .uri = "/api/v1/rotate/*",
        .method = HTTP_POST,
        .handler = rotate_control_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &rotate_control_post_uri);

    /* URI handler for file list */
    httpd_uri_t file_control_list_get_uri = {
        .uri = "/api/v1/files/list",
        .method = HTTP_GET,
        .handler = file_control_list_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &file_control_list_get_uri);

    // /* URI handler for file rename */
    // httpd_uri_t file_control_rename_post_uri = {
    //     .uri = "/api/v1/files/rename",
    //     .method = HTTP_POST,
    //     .handler = file_control_rename_handler,
    //     .user_ctx = rest_context
    // };
    // httpd_register_uri_handler(server, &file_control_rename_post_uri);

    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = rest_common_get_handler,
        .user_ctx = rest_context};
    httpd_register_uri_handler(server, &common_get_uri);

    return ESP_OK;
err_start:
    free(rest_context);
err:
    return ESP_FAIL;
}
