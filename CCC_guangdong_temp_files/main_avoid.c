

//? 从地上随机夹取

put_claw_up();
HAL_UART_Transmit(&huart3, (uint8_t*)"II", strlen("II"), 50);

//纯直线调整
is_slight_spin_and_move = 1;
tim3_count = 0;
while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
{
    slight_spin_and_move(); // 直线和圆环一起调整
    HAL_Delay(50);
}
is_slight_spin_and_move = 0;
stop();
HAL_Delay(50);

// 看中心物料的颜色
put_claw_down_near_ground();
HAL_Delay(1000);
HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50);
is_slight_spin_and_move =1;
tim3_count = 0;
while(is_slight_spin_and_move != 0) 
{
    slight_spin_and_move(); //根据色块定位
    HAL_Delay(50);
}
is_slight_spin_and_move = 0;
stop(); //根据中间物料的中心调整底盘

is_get_material_from_temp_area = 2;
is_slight_spin_and_move = 0;
// 看另一个颜色
get_and_pre_put_void(1,0);
HAL_Delay(500);
HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50);
//等待树莓派返回识别结果
while(is_get_material_from_temp_area != 3)
{
    HAL_Delay(100);
}

put_claw_up_top();
HAL_Delay(1000);
int temp_get_order[3] = {0,0,0};//本次抓取的位置顺序，123对应的是右中左
for(int i = 0 ; i < 3 ; i++)
{
    for(int j = 0 ; j < 3 ; j++)
    {
        if(target_colour[i] == material_place[j])
        {
            temp_get_order[i] = j+1;
            sprintf(temp,"ss target:%d,place:%d,i:%d,j+1:%d",target_colour[i],material_place[j],i,j+1);
            // HAL_UART_Transmit(&huart3, (uint8_t*)temp, strlen(temp), 50);
            HAL_Delay(10);
            break; 
        }
    }
}
for(int i = 0; i < 3; i++)
{
    get_and_load_openloop_with_temp_put(temp_get_order[i],target_colour[i]); // 开环抓取
}

//? 路径的规划


//从起始到二维码
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50);
    float start_move_y = 22.5; 
    float move_to_qrcode = 122.5;
    float little_back_1 = 2;
    move_all_direction_position(acceleration, open_loop_move_velocity, -start_move_x , start_move_y); 
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_to_qrcode); 
    HAL_Delay(4000);

    spin_right_90(open_loop_spin_velocity,acceleration_spin);
    free(target_colour_str); //! 不要忘记后面的延时
    HAL_Delay(1000);
    spin_right_90(open_loop_spin_velocity,acceleration_spin);
    free(target_colour_str); //! 不要忘记后面的延时
    HAL_Delay(1000);



// 从二维码到新暂存区

    float move_right_length_1 = 41; 
    float move_front_length_1 = 170;  
    move_all_direction_position(acceleration, open_loop_x_move_velocity, move_right_length_1,0);
    HAL_Delay(900);

    //先校正直线
    signle_line_adjust();
    

    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);
    HAL_Delay(3000);
    spin_right_180(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(2000); //TODO 提前发是否会干扰视觉的判断


    //TODO 待加入底盘调整



// 从新暂存区到新加工区

int move_front_length_2 = 82;    
    int move_back_length_2 = 86; 
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_2);
    HAL_Delay(2000);
    put_claw_up();//! 姿态的恢复
    arm_stretch();
    spin_right_90(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2 );
    HAL_Delay(1800);
        put_claw_up();



// 从新加工区到成品区


int move_45_length_5 = 28;
    int move_front_length_5 = 75.5;
    int move_back_length_5 = 169+2;
    int move_right_length_5 = 0.1;
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_5);
    HAL_Delay(1000);
    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();
    HAL_Delay(1500);
    spin_right_180(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(2000);


// 从成品区到到新暂存区

    int move_right_length_6 = 82.5;
    int move_right_length_7 = 89.5;

    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_6,0);
    HAL_Delay(2000);
    spin_right_90(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(1000);

    signle_line_adjust();


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_7);
    HAL_Delay(2000);

    //TODO 待加入底盘调整



// 从新暂存区到新加工区


int move_front_length_8 = 82;    
    int move_back_length_8 = 86; 
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_8);
    HAL_Delay(2000);
    put_claw_up();//! 姿态的恢复
    arm_stretch();
    spin_right_90(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_8 );
    HAL_Delay(1800);
        put_claw_up();




//从新加工区到成品区

int move_45_length_9 = 28;
    int move_front_length_9= 75.5;
    int move_back_length_9 = 169+2;
    int move_right_length_9 = 0.1;
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_9);
    HAL_Delay(1000);
    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();
    HAL_Delay(1500);
    spin_right_180(open_loop_spin_velocity,acceleration_spin);
    HAL_Delay(2000);



// 从成品区

spin_left_90(open_loop_spin_velocity,acceleration_spin);
HAL_Delay(1000);

int move_front_lenght_10 = 75.5;
int move_to_stop_place_x = 18.5;
int move_to_stop_place_y = 28;


move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_lenght_10);
HAL_Delay(2000);

move_all_direction_position(acceleration, open_loop_move_velocity, move_to_stop_place_x,move_to_stop_place_y);

//结束