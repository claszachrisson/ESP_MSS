#include "A2DP_sink.h"
#include "socket.h"

static const char *BT_AV_TAG =  "A2DP_SINK";
static const char *BT_RC_TG_TAG =   "RC_TG";
static const char *BT_RC_CT_TAG =   "RC_CT";

uint8_t reconnects = 0;
esp_bd_addr_t last_connection = {0,0,0,0,0,0};
void (*set_abs_vol_cb)(uint8_t vol);
void (*amp_data_cb)(const uint8_t *buf, uint32_t len);

const char* to_str(esp_bd_addr_t bda) {
    static char bda_str[18];
    sprintf(bda_str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2],
            bda[3], bda[4], bda[5]);
    return (const char*)bda_str;
}

esp_err_t write_bda_to_nvs(const char* name, esp_bd_addr_t bda) {
    nvs_handle my_handle;
    esp_err_t err;

    err = nvs_open("connected_bda", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "NVS OPEN ERROR");
        return false;
    }
    err = nvs_set_blob(my_handle, "last_bda", bda, ESP_BD_ADDR_LEN);
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "NVS WRITE ERROR");
    }
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "NVS COMMIT ERROR");
    }
    nvs_close(my_handle);
    return err;
}

void get_last_connection(){
    nvs_handle my_handle;
    esp_err_t err;
    
    err = nvs_open("connected_bda", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG,"NVS OPEN ERROR");
    }

    esp_bd_addr_t bda;
    size_t size = sizeof(bda);
    err = nvs_get_blob(my_handle, "last_bda", bda, &size);
    if ( err != ESP_OK) { 
        if ( err == ESP_ERR_NVS_NOT_FOUND ) {
            ESP_LOGE(BT_AV_TAG, "nvs_blob does not exist");
        } else {
            ESP_LOGE(BT_AV_TAG, "nvs_get_blob failed");
        }
    }
    nvs_close(my_handle);
    if (err == ESP_OK) {
        memcpy(last_connection,bda,size);
    } 
}

esp_err_t set_last_connection(esp_bd_addr_t bda){
    ESP_LOGD(BT_AV_TAG, "set_last_connection: %s", to_str(bda));

    // same value: nothing to store
    if (memcmp(bda, last_connection, ESP_BD_ADDR_LEN) == 0) {
        ESP_LOGD(BT_AV_TAG, "no change!");
        return ESP_OK;
    }
    esp_err_t err = write_bda_to_nvs("last_bda", bda);
    // update last_connection variable
    memcpy(last_connection, bda, ESP_BD_ADDR_LEN);
    return err;
}

bool has_last_connection() {  
    esp_bd_addr_t empty_connection = {0,0,0,0,0,0};
    int result = memcmp(last_connection, empty_connection, ESP_BD_ADDR_LEN);
    return result!=0;
}
esp_err_t reconnect(esp_bd_addr_t peer){
    if(reconnects > 2)
        return ESP_BT_STATUS_HCI_REPEATED_ATTEMPTS;
    reconnects++;
    esp_err_t err = esp_a2d_sink_connect(peer);
    if (err!=ESP_OK){
        ESP_LOGE(BT_AV_TAG, "esp_a2d_sink_connect:%d", err);
        return err;
    }
    reconnects = 0;
    return err;
}

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_DEV_NAME_RES_EVT: {
        if (param->name_res.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "Get local device name success: %s", param->name_res.name);
        } else {
            ESP_LOGE(BT_AV_TAG, "Get local device name failed, status: %d", param->name_res.status);
        }
        break;
    }
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    uint8_t *bda = NULL;

    switch (event) {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        ESP_LOGI(BT_AV_TAG, "link key type of current link is: %d", param->auth_cmpl.lk_type);
        break;
    }
    case ESP_BT_GAP_ENC_CHG_EVT: {
        const char *str_enc[3] = {"OFF", "E0", "AES"};
        bda = static_cast<uint8_t*>(param->enc_chg.bda);
        ESP_LOGI(BT_AV_TAG, "Encryption mode to [%02x:%02x:%02x:%02x:%02x:%02x] changed to %s",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], str_enc[param->enc_chg.enc_mode]);
        break;
    }

    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d, interval: %.2f ms",
                param->mode_chg.mode, param->mode_chg.interval * 0.625);
        break;
    /* when ACL connection completed, this event comes */
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to [%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);
        set_last_connection(bda);
        break;
    /* when ACL disconnection completed, this event comes */
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_disconn_cmpl_stat.reason);
        break;
    /* others */
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);


    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = param->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                param->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        break;
    }
    /* when passthrough response, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d", param->psth_rsp.key_code,
                    param->psth_rsp.key_state, param->psth_rsp.rsp_code);
        break;
    }
    /* when metadata response, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", param->meta_rsp.attr_id, param->meta_rsp.attr_text);
        free(param->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d", param->change_ntf.event_id);
        //bt_av_notify_evt_handler(param->change_ntf.event_id, &param->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %"PRIx32", TG features %x", param->rmt_feats.feat_mask, param->rmt_feats.tg_feat_flag);
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", param->get_rn_caps_rsp.cap_count,
                param->get_rn_caps_rsp.evt_set.bits);
        /*s_avrc_peer_rn_cap.bits = param->get_rn_caps_rsp.evt_set.bits;
        bt_av_new_track();
        bt_av_playback_changed();
        bt_av_play_pos_changed();*/
        break;
    }
    case ESP_AVRC_CT_COVER_ART_STATE_EVT: {
        break;
    }
    case ESP_AVRC_CT_COVER_ART_DATA_EVT: {
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}
void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        uint8_t *bda = param->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_TG_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                param->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        /*if (param->conn_stat.connected) {
            // create task to simulate volume change
            xTaskCreate(volume_change_simulation, "vcsTask", 2048, NULL, 5, &s_vcs_task_hdl);
        } else {
            vTaskDelete(s_vcs_task_hdl);
            ESP_LOGI(BT_RC_TG_TAG, "Stop volume change simulation");
        }*/
        break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC passthrough cmd: key_code 0x%x, key_state %d", param->psth_cmd.key_code, param->psth_cmd.key_state);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC set absolute volume: %d%%", (int)param->set_abs_vol.volume * 100 / 0x7f);
        set_abs_vol_cb(param->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC register event notification: %d, param: 0x%"PRIx32, param->reg_ntf.event_id, param->reg_ntf.event_parameter);
        /*if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }*/
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC remote features: %"PRIx32", CT features: %x", param->rmt_feats.feat_mask, param->rmt_feats.ct_feat_flag);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}



// Bluetooth initialization
void bt_init() {
    esp_err_t err;

    /*
    * This example only uses the functions of Classical Bluetooth.
    * So release the controller memory for Bluetooth Low Energy.
    */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialize and enable Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }

    // Initialize Bluedroid (Bluetooth host stack)
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;
    if ((err = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    /* set default parameters for Legacy Pairing (use fixed pin code) */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
}

// A2DP event callback
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            ESP_LOGI(BT_AV_TAG, "A2DP connection state: %d", param->conn_stat.state);
            if(param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED){
                //reconnect(last_connection);
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            ESP_LOGI(BT_AV_TAG, "A2DP audio state: %d", param->audio_stat.state);
            break;

        case ESP_A2D_AUDIO_CFG_EVT:
            ESP_LOGI(BT_AV_TAG, "A2DP audio codec configured");
            if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
                uint8_t *sbc_cfg = param->audio_cfg.mcc.cie.sbc;
                ESP_LOGI(BT_AV_TAG, "Configure SBC codec:");
                ESP_LOGI(BT_AV_TAG, "Sample rate | channel mode: %d", sbc_cfg[0]);
                ESP_LOGI(BT_AV_TAG, "Block length | subbands: %d", sbc_cfg[1]);
                ESP_LOGI(BT_AV_TAG, "Min bitpool: %d", sbc_cfg[2]);
                ESP_LOGI(BT_AV_TAG, "Max bitpool: %d", sbc_cfg[3]);

                // Modify the SBC parameters here
                //sbc_cfg[4] = 2;  // Min bitpool
                //sbc_cfg[5] = 53; // Max bitpool (highest quality)
            }
            break;

        default:
            ESP_LOGI(BT_AV_TAG, "Unhandled A2DP event: %d", event);
            break;
    }
}
void dummy_vol_cb(uint8_t vol){}

void a2dp_sink_register_amp_data_cb(void (*callback)(const uint8_t *buf, uint32_t len)) {
    amp_data_cb = callback;
}
void a2dp_sink_data_cb(const uint8_t *buf, uint32_t len) {
    multicast_send(buf,len);
    amp_data_cb(buf,len);
}


// A2DP sink initialization
void a2dp_sink_init() {

    esp_log_level_set("BT_BTC", ESP_LOG_INFO); // ESP_LOG_VERBOSE);
    esp_log_level_set("BT_APPL", ESP_LOG_INFO); // ESP_LOG_VERBOSE);
    esp_log_level_set("BT_HCI", ESP_LOG_INFO); // ESP_LOG_VERBOSE);

    set_abs_vol_cb = dummy_vol_cb;
    // Initialize Bluetooth
    bt_init();

    // Set Bluetooth device name
    esp_bt_gap_set_device_name("ESP32_A2DP_SINK");

    esp_bt_dev_register_callback(bt_app_dev_cb);
    esp_bt_gap_register_callback(bt_app_gap_cb);

    assert(esp_avrc_ct_init() == ESP_OK);
    esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
    assert(esp_avrc_tg_init() == ESP_OK);
    esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

    esp_avrc_rn_evt_cap_mask_t evt_set = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_sink_register_data_callback(a2dp_sink_data_cb);
    esp_a2d_sink_init();


    // Enable discoverability and connectability
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        get_last_connection();

        if(has_last_connection()){
            reconnect(last_connection);
        }
    }
}

void a2dp_sink_register_abs_vol_cb(void (*callback)(uint8_t vol)) {
    set_abs_vol_cb = callback;
}