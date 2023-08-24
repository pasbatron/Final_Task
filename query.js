let latitude = msg.payload.publish_e_v_ta_latitude;
let longitude = msg.payload.publish_e_v_ta_longitude;
let altitude = msg.payload.publish_e_v_ta_altitude;
let gyro_aks_x = msg.payload.publish_e_v_ta_gyro_akselerasi_x;
let gyro_aks_y = msg.payload.publish_e_v_ta_gyro_akselerasi_y;
let gyro_aks_z = msg.payload.publish_e_v_ta_gyro_akselerasi_z;
let gyro_rts_x = msg.payload.publish_e_v_ta_gyro_rotasi_x;
let gyro_rts_y = msg.payload.publish_e_v_ta_gyro_rotasi_y;
let gyro_rts_z = msg.payload.publish_e_v_ta_gyro_rotasi_z;
let gyro_temperature = msg.payload.publish_e_v_ta_gyro_temperature;

msg.topic = 'INSERT INTO tabel_data(id, longitude, latitude, altitude, akselerasi_x, akselerasi_y, akselerasi_z, rotasi_x, rotasi_y, rotasi_z, suhu_gyro, tanggal, waktu, alat) values(null, $val1, $val2, $val3, $val4, $val5, $val6, $val7, $val8,$val9,$val10, date("now"),time("now"), "Alat-Params");'
msg.payload = [longitude, latitude, altitude, gyro_aks_x, gyro_aks_y, gyro_aks_z, gyro_rts_x, gyro_rts_y, gyro_rts_z, gyro_temperature]

return msg;
