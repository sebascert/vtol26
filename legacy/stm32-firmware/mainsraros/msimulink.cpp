#include <Arduino.h>
#include <ICM42688.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Servo.h> 


#define LED_PIN PA14
#define IMU_CS PA4

ICM42688 imu(SPI, IMU_CS);

TinyGPSPlus gps;
float lat = 0, lon = 0;
bool gps_valid = false;

Adafruit_BMP280 bmp;
float altitude = 0;

Servo motor1, motor2, motor3;

uint32_t t_imu     = 0;
uint32_t t_control = 0;
uint32_t t_esc     = 0;
uint32_t t_sensores = 0;

//Filtro 

float ax, ay, az; //IMU Variables
float gx, gy, gz;

float roll = 0; //
float pitch = 0;

float alpha = 0.98;
float dt = 0.002; // 500 Hz

struct SimData { // Binario para HILS
    uint8_t header;     // 0xAA
    float ax, ay, az;   // aceleraciones
    float gx, gy, gz;   // giros
    float altitude;     // altitud simulada
    uint8_t checksum;   // suma simple
};

SimData sim;
bool modoHILS = false; 

struct OutData {
    uint8_t header;   // 0xBB
    float roll;
    float pitch;
    float pwm1;
    float pwm2;
    float pwm3;
    uint8_t checksum;
};

void enviarASimulink(float roll, float pitch, float pwm1, float pwm2, float pwm3) {
    OutData out;
    out.header = 0xBB;
    out.roll = roll;
    out.pitch = pitch;
    out.pwm1 = pwm1;
    out.pwm2 = pwm2;
    out.pwm3 = pwm3;

    uint8_t *ptr = (uint8_t*)&out;
    uint8_t sum = 0;
    for (int i = 0; i < sizeof(OutData)-1; i++) sum += ptr[i];
    out.checksum = sum;

    Serial.write((uint8_t*)&out, sizeof(OutData));
}

void leerSimulink() {
    if (Serial.available() >= sizeof(SimData)) {
        Serial.readBytes((char*)&sim, sizeof(SimData));

        if (sim.header == 0xAA) {
            uint8_t sum = 0;
            uint8_t *ptr = (uint8_t*)&sim;
            for (int i = 0; i < sizeof(SimData)-1; i++) sum += ptr[i];

            if (sum == sim.checksum) {
                modoHILS = true;
                ax = sim.ax; ay = sim.ay; az = sim.az;
                gx = sim.gx; gy = sim.gy; gz = sim.gz;
                altitude = sim.altitude;
            }
        }
    }
}

void tarea_imu() {
//leer datos 
    if (!modoHILS){ //Para Simulink
    
        imu.getAGT();

        ax = imu.accX();
        ay = imu.accY();
        az = imu.accZ();

        gx = imu.gyrX();
        gy = imu.gyrY();
        gz = imu.gyrZ();
    }
}

void tarea_control() { 
    //Ángulo acelerómetro /// 
    float roll_acc  = atan2(ay, az);
    float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));

    //Filtro complementario
    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * roll_acc;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc;
    
}

void tarea_sensores() {
    if (!modoHILS) { //Simulink
        while (Serial1.available()) gps.encode(Serial1.read());
        if (gps.location.isValid()) {
            lat = gps.location.lat();
            lon = gps.location.lng();
            gps_valid = true;
        } else gps_valid = false;

        altitude = bmp.readAltitude(1013.25);
    }
}


void tarea_esc() {
    // 
    motor1.writeMicroseconds(1500);
    motor2.writeMicroseconds(1500);
    motor3.writeMicroseconds(1500);
}

// tarea de validación — parpadea el LED cada 500ms
void tarea_led() {
    static bool estado = false;
    estado = !estado;
    digitalWrite(LED_PIN, estado);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);   
    Serial1.begin(9600); 

    while(!Serial);      
    delay(2000);
    

    // Inicializar SPI e IMU
    SPI.begin();
    int status = imu.begin();
    if (status < 0) {
      Serial.println("ERROR: IMU no detectado");
      while (1);
    }

    Serial.println("Detecta IMU");

    //BARO
    if (!bmp.begin(0x76)) { // prueba 0x77 si falla
        Serial.println("ERROR: BARO");
        while (1);
    }
    Serial.println("Detecta BARO");
    motor1.attach(PA0); // salida PWM
    motor2.attach(PA1);
    motor3.attach(PA2);

}

void loop() {
    leerSimulink(); // revisar si hay datos simulados
    uint32_t ahora = micros();

    if (ahora - t_imu >= 1000) {          // cada 1ms   = 1000 Hz
        t_imu = ahora;
        tarea_imu();
    }

    if (ahora - t_control >= 2000) {      // cada 2ms   =  500 Hz
        t_control = ahora;
        tarea_control();
    }

    if (ahora - t_esc >= 2500) {          // cada 2.5ms =  400 Hz
        t_esc = ahora;
        tarea_esc();
    }

       // GPS + BARO (100 Hz)
    if (ahora - t_sensores >= 10000) {
        t_sensores = ahora;
        tarea_sensores();
        }

    
    //    
    static uint32_t t_debug = 0;
    if (millis() - t_debug > 200) {
        t_debug = millis();

        Serial.print("Roll: "); Serial.print(roll);
        Serial.print(" Pitch: "); Serial.print(pitch);
        Serial.print(" Alt: "); Serial.print(altitude);

        if (gps_valid) {
            Serial.print(" Lat: "); Serial.print(lat);
            Serial.print(" Lon: "); Serial.print(lon);
        }

        Serial.println();

         // enviar salidas a Simulink
        enviarASimulink(roll, pitch, 1500, 1500, 1500);
    }


    // tarea de validación visual — separada del scheduler real
    static uint32_t t_led = 0;
    if (ahora - t_led >= 500000) {        // cada 500ms = 2 veces/seg
        t_led = ahora;
        tarea_led();
    }
}