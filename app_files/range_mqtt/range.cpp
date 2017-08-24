#include "range.h"


node_t get_node(range_data_t data){
    return {data.node_id, data.tdoa};
}


int load_data(char *buff, int buff_size, node_t node){
    if((num_nodes_to_pub + 1) * DATA_STRING_SIZE >= buff_size - 1){
        PRINTF("Buffer is full\n");
        return -1;
    }
    snprintf(buff + (num_nodes_to_pub * DATA_STRING_SIZE), buff_size - (num_nodes_to_pub * DATA_STRING_SIZE), "%05d,%02d;", node.tdoa, node.node_id);
    num_nodes_to_pub++;
    PRINTF("# of nodes = %d\n",num_nodes_to_pub);
    return 0;
}

void clear_data(char *buff, int buff_size){
    memset(buff,0,buff_size);
    num_nodes_to_pub = 0;
}

dist_angle_t get_dist_angle(range_data_t *time_diffs, uint8_t ranging_mode){
    int tdoa_a;
    int tdoa_b;
    float dist_a;
    float dist_b;
    float dist;
    float angle = -361;

    dist_angle_t return_val;
    return_val.distance = -1;
    return_val.angle = 0;
    return_val.node_id = 0;

    tdoa_a = time_diffs->tdoa;
    dist_a = get_dist(tdoa_a);

    switch (ranging_mode)
    {
        case ONE_SENSOR_MODE:
            dist = dist_a;
            break;
        case TWO_SENSOR_MODE:
            if(time_diffs->status > 2)
            {
                PRINTF("Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs-> status); 
            } 
            else
            {
                tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
                dist_b = get_dist(tdoa_b);
                PRINTF("OD = %lu\n", time_diffs-> orient_diff);
            }
            break;
        case XOR_SENSOR_MODE:
            tdoa_b = time_diffs->tdoa + time_diffs->orient_diff;
            dist_b = get_dist(tdoa_b);
            PRINTF("OD = %lu\n", time_diffs->orient_diff);
            break;
        case OMNI_SENSOR_MODE:
            dist = dist_a;
            break;
    }
    if(dist_b != 0){
        dist = get_mid_dist(dist_a, dist_b);
        if(time_diffs->status == 2){
            angle = get_angle(dist_b, dist_a);
        }
        else{
            angle = get_angle(dist_a, dist_b);
        }
        PRINTF("Distance: %.2f\n", dist);
        PRINTF("Angle : %.2f\n", angle);
    }
    else{
        PRINTF("Distance: %.2f\n", dist);
    }
     printf("******************************\n");

    return_val.distance = dist;
    return_val.angle = angle;
    return_val.node_id = time_diffs->node_id;

    printf("Range: Returning value\n");
    printf("Range: %.2f, %.2f, %d\n",return_val.distance,return_val.angle,return_val.node_id);
    return return_val;
}

range_data_t get_range_data(range_params_t params){
    ranging = 1;
 
    int exit = 0;
    int pkt_size = sizeof(uart_pkt_hdr_t) + sizeof(range_params_t);
    char send_data[pkt_size];

    msg_t *msg, *msg2;
    hdlc_buf_t *buf;
    hdlc_pkt_t pkt;
    
    uart_pkt_hdr_t recv_hdr;
    uart_pkt_hdr_t send_hdr = (uart_pkt_hdr_t){ MBED_RANGE_PORT, RIOT_MQTT_PORT, SOUND_RANGE_REQ };
    
    /* misc */
    osEvent evt;

    range_data_t* time_diffs;
    range_hdr_t* range_hdr;
    
    int i = 0;
    int j = 0;
    int data_per_pkt;

    //send ranging request packet up hdlc   
    pkt.data = send_data;
    pkt.length = pkt_size;   
    uart_pkt_insert_hdr(pkt.data, pkt.length, &send_hdr); 
    uart_pkt_cpy_data(pkt.data, pkt.length, &params, sizeof(range_params_t));

    PRINTF("src_port: %d, dst_port: %d\n", send_hdr.src_port, send_hdr.dst_port);
    PRINTF("node_id: %d\n", params.node_id);

    if (send_hdlc_mail(msg, HDLC_MSG_SND, &range_thr_mailbox, (void*) &pkt)){
        PRINTF("range_thread: sending range_req pkt\n"); 
    }
    else{
        PRINTF("range_thread: failed to send pkt no\n"); 
        return {0,0,0,params.node_id};
    }
    //recieving data
    while(!exit)
    {
        PRINTF("range_thread: Waiting for response\n");
        evt = range_thr_mailbox.get();
        
        if(evt.status == osEventMail)
        {
            PRINTF("range_thread: Got a response\n");
            msg = (msg_t*)evt.value.p;

            switch (msg->type)
            {
                case HDLC_RESP_SND_SUCC:
                    PRINTF("range_thread: sent frame!\n");
                    range_thr_mailbox.free(msg);
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    Thread::wait(msg->content.value/1000);
                    PRINTF("range_thread: retrying\n");

                    /* Tries to allocate memory for msg2. */
                    msg2 = hdlc_mailbox_ptr->alloc();
                    if(msg2 == NULL) 
                    {
                        /* Blocking call until the memory has been allocated. */
                        // Thread::wait(50);
                        while(msg2 == NULL)
                        {
                            msg2 = range_thr_mailbox.alloc();  
                            Thread::wait(10);
                        }
                        msg2->type = HDLC_RESP_RETRY_W_TIMEO;
                        msg2->content.value = (uint32_t) RTRY_TIMEO_USEC;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &range_thr_mailbox;
                        range_thr_mailbox.put(msg2);
                    }
                    else
                    {
                        msg2->type = HDLC_MSG_SND;
                        msg2->content.ptr = &pkt;
                        msg2->sender_pid = osThreadGetId();
                        msg2->source_mailbox = &range_thr_mailbox;
                        hdlc_mailbox_ptr->put(msg2);
                    }
                    range_thr_mailbox.free(msg);
                    break;

                case HDLC_PKT_RDY:
                    /* Setting up buf, making it easier to access data from the msg. */ 
                    buf = (hdlc_buf_t *)msg->content.ptr;   
                    uart_pkt_parse_hdr(&recv_hdr, buf->data, buf->length);

                    if(recv_hdr.pkt_type == SOUND_RANGE_DONE) 
                    {
                        PRINTF("range_thread: received range pkt\n");
                        range_hdr = (range_hdr_t *)uart_pkt_get_data(buf->data, buf->length);
                        time_diffs = (range_data_t *)range_hdr->data;
                        
                        data_per_pkt = (buf->length - sizeof(uart_pkt_hdr_t) - sizeof(uint8_t))/sizeof(range_data_t);
                        PRINTF("range_thread: There should be %d ranges in this pkt\n",data_per_pkt);

                        if(data_per_pkt == 0){
                            ranging = 0;
                            hdlc_pkt_release(buf);
                            range_thr_mailbox.free(msg);
                            return (range_data_t){0,0,0,params.node_id};
                        }

                        for(i = 0; i < data_per_pkt; i++){
                            PRINTF ("%d:\n", i);
                            /* Displaying results. */
                            
                            PRINTF("TDoA = %lu\n", time_diffs->tdoa);

                            switch (params.ranging_mode)
                            {
                                case ONE_SENSOR_MODE:
                                    PRINTF("One Sensor Mode^\n");
                                    break;
                                case TWO_SENSOR_MODE:
                                    if(time_diffs->status > 2){
                                        printf("Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs->status); 
                                    } 
                                    else{
                                        PRINTF("OD = %lu\n", time_diffs->orient_diff);
                                    }
                                    PRINTF("Two Sensor Mode^\n");
                                    break;
                                case XOR_SENSOR_MODE:
                                    PRINTF("OD = %lu\n", time_diffs->orient_diff);
                                    PRINTF("Xor Sensor Mode^\n");
                                    break;
                                case OMNI_SENSOR_MODE:
                                    PRINTF("Omni Sensor Mode^\n");
                                    break;
                            }

                            if(params.node_id == -1){
                                nodes_reached[j] = (node_t) {time_diffs->node_id, time_diffs->tdoa};
                                j++;
                                if(j >= MAX_NUM_ANCHORS){
                                    printf("Exceeded max number of anchors\n");
                                    return (range_data_t){0,0,0,params.node_id};
                                }
                                num_nodes_reached = j;
                            }
                            time_diffs++;
                        }
                        if(range_hdr->last_pkt == 1){
                            printf("All data recieved\n");
                            exit = 1;
                        }
                    }
                    else
                    {
                        printf("main_thr: recieved non-range pkt\n");
                        exit = 1;
                    }
                    hdlc_pkt_release(buf);
                    range_thr_mailbox.free(msg);
                    break;
                default:
                    PRINTF("Reached default case\n");
                    /* error */
                    //LED3_ON;
                    range_thr_mailbox.free(msg);
                    break;
            }
        } 
        else{
            printf("Range: Didn't get mail: %02x\n",evt.status);
        }   
        if(exit) 
        {
            printf("Range: Exiting loop\n");
            exit = 0;
            break;
        }
    }
    
    ranging = 0;

    return *(time_diffs-1);
}

void range_all(uint8_t ranging_mode){
    get_range_data((range_params_t){-1, ranging_mode});
}

range_data_t range_node(range_params_t params){
    return get_range_data(params);
}

uint16_t lock_on_anchor(int8_t node_id){
    range_data_t raw_data; 
    dist_angle_t conv_data;
    float angle;

    if(!init_minimu()){
        PRINTF("Failed to init minimu");
        return;
    }

    calibrate_compass();

    while(angle > 5 && angle < -5){
        
        raw_data = range_node({node_id, TWO_SENSOR_MODE});
        if(raw_data.tdoa = 0){
            PRINTF("Locking failed: Anchor node unavailable\n");
            return 0;
        }
        conv_data = get_dist_angle(raw_data);
        angle = conv_data.angle;

        if(angle == -361){
            rotate_degrees(180,40);
        }
        else{
            if(angle < 75 && angle > -75){
                rotate_parts(angle);
            }
            else{
                rotate_degrees(angle,40);
            }
        }
    }
    return range_data.tdoa;
}

/**
 * @brief      This is the range thread that triggers the range routine for localization.
 *             To trigger localization, a msg of type START_RANGE_THR must be sent to the
 *             range_thread_mailbox containing the parameters for localization.
 */
void _range_thread(){

    char            data_pub[32];
    hdlc_pkt_t      pkt;

    int i = 0;
    
    pkt.data = data_pub;
    pkt.length = 32;

    mqtt_pkt_t      mqtt_send;
    osEvent evt;
    msg_t *msg;

    range_data_t range_data;
    range_params_t range_params;

    hdlc_entry_t range_thread = { NULL, MBED_RANGE_PORT, &range_thr_mailbox };
    hdlc_register(&range_thread);

    while(1)
    {

        PRINTF("range_thread: Waiting for start message\n");
        evt = range_thr_mailbox.get();
        if(evt.status == osEventMail)
        {
            PRINTF("range_thread: got mail\n");
            msg = (msg_t*)evt.value.p;
            if(msg->type == START_RANGE_THR){ 

                //***This is where the range routine will go*****
                
                PRINTF("range_thread: got range init message, starting routine\n");
                range_params = *(range_params_t*)(msg->content.ptr);

                if(range_params.node_id == -1){
                    range_all(range_params.ranging_mode);
                    printf("****************Discovery mode***************\n");
                    printf("Nodes reached:\n");
                    for(i=0; i<num_nodes_reached; i++){
                        printf("Node %d: %lu\n", nodes_reached[i].node_id, nodes_reached[i].tdoa);
                    }
                    printf("*********************************************\n");
                }
                else{
                    if(range_params.ranging_mode == TWO_SENSOR_MODE){
                        range_data = lock_on_anchor(range_params.node_id);
                    }
                    else{
                         range_data = range_node(range_params);
                    }

                    load_data(data_pub, 32, get_node(range_data));                         

                    build_mqtt_pkt_pub(RANGE_TOPIC, data_pub, MBED_MQTT_PORT, &mqtt_send, &pkt); 

                    //for some reason it will only publish if you include a print statement here
                    printf("tdoa = %lu\n",range_data.tdoa);
                    printf("node_id = %lu\n",range_data.node_id);
                    PRINTF("range_thread: range_routine done. publishing data now\n");

                    if (send_hdlc_mail(msg, HDLC_MSG_SND, &mqtt_thread_mailbox, (void*) &pkt)){
                        PRINTF("mqtt_thread: sending pkt of size\n"); 
                    }
                    else{

                        PRINTF("mqtt_thread: failed to send pkt no\n"); 
                    }
                    clear_data(data_pub, 32);
                }
                

                //*************************************************
            }
            else{
                PRINTF("range_thread: Recieved something other than start message\n");
                continue;
            }
        } 
        else{
            printf("range_thread: Didn't get mail: %02x\n",evt.status);
            continue;
        }  
    }
    
}

void init_range_thread(){
    range_thr.start(_range_thread);
}

void trigger_range_routine(range_params_t params, msg_t* msg){
    msg = range_thr_mailbox.alloc();
    msg->type = START_RANGE_THR;
    msg->content.ptr = &params;
    msg->sender_pid = osThreadGetId();
    msg->source_mailbox = &mqtt_thread_mailbox;
    range_thr_mailbox.put(msg);
}

bool is_ranging(){
    return ranging;
}

node_t* get_nodes_reached(){
    return nodes_reached;
}

uint8_t get_num_nodes_reached(){
    return num_nodes_reached;
}