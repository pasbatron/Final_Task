[
    {
        "id": "a87d6db468e140d9",
        "type": "tab",
        "label": "T",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "cfd13b593d9f19bb",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_akselerasi_x",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 840,
        "y": 380,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "8e48f52222de25cf",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_akselerasi_y",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 840,
        "y": 420,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "f9b7029f26d5dc44",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_akselerasi_z",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 840,
        "y": 460,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "9ee2122c686f69f0",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_rotasi_x",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 500,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "b097d821a9fb3bd8",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_rotasi_y",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 540,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "25746d16180df351",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_rotasi_z",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 580,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "a503d7f8076c6eb2",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gyro_temperature",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 840,
        "y": 620,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "537063abab1a95e7",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_latitude",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 460,
        "y": 140,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "7bf88b300eb6a1ec",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_longitude",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 470,
        "y": 340,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "2b73b2f4b3746a08",
        "type": "debug",
        "z": "a87d6db468e140d9",
        "name": "debug 35",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 460,
        "y": 20,
        "wires": []
    },
    {
        "id": "ccca8f97bcfce271",
        "type": "join",
        "z": "a87d6db468e140d9",
        "name": "",
        "mode": "custom",
        "build": "object",
        "property": "payload",
        "propertyType": "msg",
        "key": "topic",
        "joiner": "\\n",
        "joinerType": "str",
        "accumulate": false,
        "timeout": "1",
        "count": "39",
        "reduceRight": false,
        "reduceExp": "",
        "reduceInit": "",
        "reduceInitType": "num",
        "reduceFixup": "",
        "x": 310,
        "y": 40,
        "wires": [
            [
                "32557b8828d75b21",
                "2b73b2f4b3746a08"
            ]
        ]
    },
    {
        "id": "32557b8828d75b21",
        "type": "function",
        "z": "a87d6db468e140d9",
        "name": "query",
        "func": "let gps_enc             = msg.payload.publish_e_v_ta_gps_enc;\nlet gps_date            = msg.payload.publish_e_v_ta_gps_date;\nlet gps_year            = msg.payload.publish_e_v_ta_gps_year;\nlet gps_month           = msg.payload.publish_e_v_ta_gps_month;\nlet gps_day             = msg.payload.publish_e_v_ta_gps_day;\nlet gps_time            = msg.payload.publish_e_v_ta_gps_time;\nlet gps_time_hour       = msg.payload.publish_e_v_ta_gps_time_hour;\nlet gps_time_min        = msg.payload.publish_e_v_ta_gps_time_min;\nlet gps_time_sec        = msg.payload.publish_e_v_ta_gps_time_sec;\nlet gps_time_cen_sec    = msg.payload.publish_e_v_ta_gps_time_cen_sec;\nlet gps_speed           = msg.payload.publish_e_v_ta_gps_speed;\nlet gps_speed_knots     = msg.payload.publish_e_v_ta_gps_speed_knots;\nlet gps_speed_mph       = msg.payload.publish_e_v_ta_gps_speed_mph;\nlet gps_speed_mps       = msg.payload.publish_e_v_ta_gps_speed_mps;\nlet gps_speed_kmph      = msg.payload.publish_e_v_ta_gps_speed_kmph;\nlet gps_course_raw      = msg.payload.publish_e_v_ta_gps_cour_raw;\nlet gps_course_raw_deg  = msg.payload.publish_e_v_ta_gps_cour_raw_deg;\nlet gps_satelite        = msg.payload.publish_e_v_ta_gps_satelite;\nlet gps_hdop            = msg.payload.publish_e_v_ta_gps_hdop;\nlet latitude            = msg.payload.publish_e_v_ta_latitude;\nlet latitude_raw        = msg.payload.publish_e_v_ta_gps_lat_raw;\nlet latitude_raw_deg    = msg.payload.publish_e_v_ta_gps_lat_raw_deg;\nlet latitude_raw_bill   = msg.payload.publish_e_v_ta_gps_lat_raw_bill;\nlet longitude           = msg.payload.publish_e_v_ta_longitude;\nlet longitude_raw       = msg.payload.publish_e_v_ta_gps_long_raw;\nlet longitude_raw_deg   = msg.payload.publish_e_v_ta_gps_long_deg;\nlet longitude_raw_bill  = msg.payload.publish_e_v_ta_gps_long_bill;\nlet altitude            = msg.payload.publish_e_v_ta_altitude;\nlet altitude_meter      = msg.payload.publish_e_v_ta_gps_alt_meter;\nlet altitude_mil        = msg.payload.publish_e_v_ta_gps_alt_mil;\nlet altitude_km         = msg.payload.publish_e_v_ta_gps_alt_km;\nlet altitude_feet       = msg.payload.publish_e_v_ta_gps_alt_feet;\nlet gyro_aks_x          = msg.payload.publish_e_v_ta_gyro_akselerasi_x;\nlet gyro_aks_y          = msg.payload.publish_e_v_ta_gyro_akselerasi_y;\nlet gyro_aks_z          = msg.payload.publish_e_v_ta_gyro_akselerasi_z;\nlet gyro_rts_x          = msg.payload.publish_e_v_ta_gyro_rotasi_x;\nlet gyro_rts_y          = msg.payload.publish_e_v_ta_gyro_rotasi_y;\nlet gyro_rts_z          = msg.payload.publish_e_v_ta_gyro_rotasi_z;\nlet gyro_temperature    = msg.payload.publish_e_v_ta_gyro_temperature;\n\nmsg.topic = 'INSERT INTO table_data(id, gps_encode, gps_date, gps_year, gps_month, gps_day, gps_time, gps_time_hour, gps_time_minute, gps_time_second, gps_time_centi_second, gps_speed, gps_speed_knots, speed_mph, speed_mps, speed_kmph, gps_course_raw, gps_course_raw_deg, gps_satelite, gps_hdop, latitude, latitude_raw, latitude_raw_deg, latitude_raw_bill, longitude, longitude_raw, longitude_raw_deg, longitude_raw_bill, altitude, altitude_meter, altitude_mil, altitude_km, altitude_feet, akseleration_x, akseleration_y, akseleration_z, rotation_x, rotation_y, rotation_z, gyro_temp, date, time, device) values(null, $val1, $val2, $val3, $val4, $val5, $val6, $val7, $val8, $val9, $val10, $val11, $val12, $val13, $val14, $val15, $val16, $val17, $val18, $val19, $val20, $val21, $val22, $val23, $val24, $val25, $val26, $val27, $val28, $val29, $val30, $val31, $val32, $val33, $val34, $val35, $val36, $val37, $val38, $val39, date(\"now\"),time(\"now\"), \"Alat-Params\");'\nmsg.payload = [gps_enc, gps_date, gps_year, gps_month, gps_day, gps_time, gps_time_hour, gps_time_min, gps_time_sec, gps_time_cen_sec, gps_speed, gps_speed_knots, gps_speed_mph, gps_speed_mps, gps_speed_kmph, gps_course_raw, gps_course_raw_deg, gps_satelite, gps_hdop, latitude, latitude_raw, latitude_raw_deg, latitude_raw_bill, longitude, longitude_raw, longitude_raw_deg, longitude_raw_bill, altitude, altitude_meter, altitude_mil, altitude_km, altitude_feet, gyro_aks_x, gyro_aks_y, gyro_aks_z, gyro_rts_x, gyro_rts_y, gyro_rts_z, gyro_temperature]\n\nreturn msg;",
        "outputs": 1,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 450,
        "y": 60,
        "wires": [
            [
                "1c30e3bfdd1eac7f"
            ]
        ]
    },
    {
        "id": "0e185e33710342d1",
        "type": "comment",
        "z": "a87d6db468e140d9",
        "name": "",
        "info": "Plot data kecepatan terhadap waktu, \nplot data kemiringan terhadap waktu dan jarak, \nplot data jarak terhadap waktu, \n\nplot data konsumsi baterai terhadap waktu dan jarak",
        "x": 100,
        "y": 60,
        "wires": []
    },
    {
        "id": "1c30e3bfdd1eac7f",
        "type": "sqlite",
        "z": "a87d6db468e140d9",
        "mydb": "7bf32b52225edc02",
        "sqlquery": "msg.topic",
        "sql": "",
        "name": "TA_Database",
        "x": 600,
        "y": 60,
        "wires": [
            []
        ]
    },
    {
        "id": "f6316c3f463818e0",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_altitude",
        "qos": "2",
        "datatype": "utf8",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 800,
        "y": 140,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "727a3729d03d8b37",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_enc",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 140,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "c313b25eb60f74e5",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_date",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 180,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "f7c469c93bb97382",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_year",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 220,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "28a23bdb02a98f53",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_month",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 160,
        "y": 260,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "256d9dd54c653383",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_day",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 300,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "1bae9ff6d84c4d08",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_time",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 340,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "f74dbe3b82e29e97",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_time_hour",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 170,
        "y": 380,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "7e516462fbe6e5a9",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_time_min",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 160,
        "y": 420,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "3c7e6e3fe48a38b5",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_time_sec",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 160,
        "y": 460,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "9e534c99ae9ab412",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_time_cen_sec",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 180,
        "y": 500,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "2cf35ec3ac35beb3",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_lat_raw",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 480,
        "y": 180,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "592d874e36f92adf",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_lat_raw_deg",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 490,
        "y": 220,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "cded48faa2182f5b",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_lat_raw_bill",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 490,
        "y": 260,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "c3902693f7f832ab",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_long_raw",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 480,
        "y": 380,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "13e6ac978cac66b5",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_long_deg",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 480,
        "y": 420,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "514c71906db173fe",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_long_bill",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 480,
        "y": 460,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "d36d5213aa53badb",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_alt_meter",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 830,
        "y": 180,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "f8be6c3bb1c0bf1c",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_alt_mil",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 220,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "2f578d3c071b6c3b",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_alt_km",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 260,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "6e9c09b44d0678f5",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_alt_feet",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 820,
        "y": 300,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "4dbe731b5f6d53a6",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_speed",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 540,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "b0b4d82437f84b53",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_speed_knots",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 180,
        "y": 580,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "f0956b4845862826",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_speed_mph",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 170,
        "y": 620,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "69138295ce9bb510",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_speed_mps",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 170,
        "y": 660,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "c0335bc594f13cfc",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_speed_kmph",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 180,
        "y": 700,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "38a48a2732471154",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_cour_raw",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 160,
        "y": 740,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "df5703b5690c8145",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_cour_raw_deg",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 180,
        "y": 780,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "c81927f9f3de530f",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_satelite",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 160,
        "y": 820,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "cf94971b48fa50c3",
        "type": "mqtt in",
        "z": "a87d6db468e140d9",
        "name": "",
        "topic": "publish_e_v_ta_gps_hdop",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "b7fb6e4682c52d9e",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 150,
        "y": 860,
        "wires": [
            [
                "ccca8f97bcfce271"
            ]
        ]
    },
    {
        "id": "b7fb6e4682c52d9e",
        "type": "mqtt-broker",
        "name": "",
        "broker": "broker.mqtt-dashboard.com",
        "port": "1883",
        "clientid": "",
        "autoConnect": true,
        "usetls": false,
        "protocolVersion": "4",
        "keepalive": "60",
        "cleansession": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closeQos": "0",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    },
    {
        "id": "7bf32b52225edc02",
        "type": "sqlitedb",
        "db": "D:\\Tugas Akhir\\Alat_Ukur\\Database\\tugas_akhir.db",
        "mode": "RWC"
    }
]
