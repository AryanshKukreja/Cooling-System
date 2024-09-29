// Define pin for PWM output
const int pwmPin = 9;

// PID constants
const float Kp = 0.01;
const float Ki = 0.001;
const float Kd = 0.00001;
const float setpoint = 35.0;

// PID variables
float integral = 0;
float previous_error = 0;
const float time_step = 1.0;// in seconds

// Energy usage variable
double net_energy_used = 0.0; // Using double for precision

// Simulation parameters
const float heat_gain_rate = 0.5; // Adjust based on your system
const float heat_expulsion_factor = 0.1; // Adjust based on fan efficiency
float fan_speed = 0.0;
// PID limits
const float maxIntegral = 10.0;

float current_temperatures[6][15];

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Introduce random initial temperatures
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 15; j++) {
      current_temperatures[i][j] = random(30, 45); // Randomize initial temperatures
    }
  }

  // Set PWM pin as output
  pinMode(pwmPin, OUTPUT);
}

void loop() {
  static float simulation_duration = 10.0; 

  float a = millis();
  
  // Simulation loop
  if (a < simulation_duration * 1000) {
    float control_signal = 0.0;
    float fs=0.0;
    // Update control signals and fan speeds for each sensor
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 15; j++) {
        // Get updated temperature
        current_temperatures[i][j] = update_temperature(fan_speed, time_step, current_temperatures[i][j]);
        // Calculate PID control signal
        control_signal = pidUpdate(current_temperatures[i][j], time_step);
        current_temperatures[i][j] += control_signal;
        fs+= control_signal;
      }
    }
    //Calculate average temperature
    float avg_temp = calculateAverageTemperature(current_temperatures);    
    Serial.print("Time: ");
    Serial.print(millis() / 1000);
    Serial.print("s - Average Temperature: ");
    Serial.println(avg_temp);
    //Serial.println(fs);
    // Map average fan speed to PWM value
    int pwm_value = mapFloat((abs(fs/90)), 0, 0.05, 0, 255);
    analogWrite(pwmPin, pwm_value);
    float dutyCycle = pwm_value / 255.0;
    //Serial.println(pwm_value);
    //Serial.println(dutyCycle);
    // Calculate and accumulate energy usage
    double power = 6.6 * dutyCycle; // Power in watts (using double for precision)
    net_energy_used += power * time_step;// Accumulate energy (power * time)
    fan_speed = dutyCycle * 32.46 ;
    }
  else {
    // Simulation completed, stop the fans
    analogWrite(pwmPin, 0);

    // Print net energy used
    Serial.print("Net Energy Used: ");
    Serial.print(net_energy_used);
    Serial.println(" Joules"); // Assuming time_step is in seconds, energy will be in Joules

    // Prevent further execution
    while (true);
  }
  float b = millis();
  float extra = 1000 - (b - a);
  delay(extra);
}

float pidUpdate(float current_value, float dt) {
  float error = setpoint - current_value;
  integral += error * dt;
  // Anti-windup: Limit integral to a maximum value
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float derivative = (error - previous_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  return output; // Output limiting
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateAverageTemperature(float temperatures[6][15]) {
  float sum = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 15; j++) {
      sum += temperatures[i][j];
    }
  }
  return sum / (6 * 15);
}

float update_temperature(float current_fanspeed, float dt, float prev_temperature) {
  float heat_expulsion_rate = heat_expulsion_factor * current_fanspeed;
  float new_temperature = prev_temperature + (heat_gain_rate - heat_expulsion_rate) * dt;
  return new_temperature;
}