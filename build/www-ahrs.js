require=(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({"./DCM":[function(require,module,exports){
'use strict';

/**
 * The DCM algorithm.  See: https://www.sparkfun.com/tutorial/news/DCMDraft2.pdf
 * @param {number} sampleInterval The sample interval in milliseconds.
 * @param {object} options
 */
module.exports = function DCM(sampleInterval, options) {
    //-- init
    // dcm.DCM_init(Kp_ROLLPITCH, Ki_ROLLPITCH, Kp_YAW, Ki_YAW);
    // reset_sensor_fusion(ax, ay, az, heading);

    //-- loop
    // dcm.G_Dt = 1./ sampleFreq;
    // dcm.calDCM(gx, gy, gz, ax, ay, az, heading, deltaTimeSec);
    // dcm.getDCM2Q(q);


    //---------------------------------------------------------------------------------------------------
    // Definitions

    options = options || {};
    var sampleFreq = 1000 / sampleInterval;  // sample frequency in Hz


    //------------------------------------------------------------
    // Variables and constants from original h file
    var GRAVITY = 9.8;//1.0;
    
    function TO_RAD(x){
        return x * Math.PI / 180;
    }

    function TO_DEG(x){
        return x * 180 / Math.PI;
    }

    // DCM timing in the main loop
    var timestamp = 0;
    var timestamp_old = 0;

    //private
    // DCM variables
    var Kp_ROLLPITCH, Ki_ROLLPITCH, Kp_YAW, Ki_YAW;
    var Omega_P = [0, 0, 0]; // Omega Proportional correction
    var Omega_I = [0, 0, 0]; // Omega Integrator
    
    var DCM_Matrix = null;   

    

    DCM_init(options.kp_rollPitch, options.ki_rollPitch, options.kp_yaw, options.ki_yaw);

    function DCM_init(Kp_RP, Ki_RP, Kp_Y, Ki_Y){

      //
      //Get Proportional and Integral Gains set in FreeIMU Library
      // 
      Kp_ROLLPITCH = Kp_RP || 1.2;
      Ki_ROLLPITCH = Ki_RP || 0.0234;
      Kp_YAW = Kp_Y || 1.75;
      Ki_YAW = Ki_Y || 0.002;

    }

    //------------------- MATH.INO --------------------

    // Computes the dot product of two vectors
    function Vector_Dot_Product(v1, v2) {
        var result = 0;
  
        for(var c = 0; c < 3; c++){
            result += v1[c] * v2[c];
        }

        return result; 
    }

    // Computes the cross product of two vectors
    // out has to different from v1 and v2 (no in-place)!
    function Vector_Cross_Product(v1, v2){
      var out = [];
      out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
      out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
      out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
      return out;
    }

    // Multiply the vector by a scalar
    function Vector_Scale(v, scale){
      var out = [];
      for(var c = 0; c < 3; c++){
        out[c] = v[c] * scale; 
      }
      return out;
    }

    // Adds two vectors
    function Vector_Add(v1, v2){
      var out = [];
      for(var c = 0; c < 3; c++)
      {
        out[c] = v1[c] + v2[c];
      }
      return out;
    }

    // Multiply two 3x3 matrices: out = a * b
    // out has to different from a and b (no in-place)!
    // a, b: 3x3 matrices
    function Matrix_Multiply(a, b){
      var out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for(var x = 0; x < 3; x++){  // rows
        for(var y = 0; y < 3; y++){  // columns
          out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
        }
      }
      return out;
    }

    // Multiply 3x3 matrix with vector: out = a * b
    // out has to different from b (no in-place)!
    // a: 3x3 matrix
    // b: 1x3 vecctor
    // out: 1x3 vector
    function Matrix_Vector_Multiply(a, b){
      var out = [];
      for(var x = 0; x < 3; x++){
        out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
      }
      return out;
    }

    // Init rotation matrix using euler angles
    // @param m 3x3 matrix to init
    // yaw, pitch, roll scalars
    function init_rotation_matrix(m, yaw, pitch, roll){
      var cr = Math.cos(roll);
      var sr = Math.sin(roll);
      var cp = Math.cos(pitch);
      var sp = Math.sin(pitch);
      var cy = Math.cos(yaw);
      var sy = Math.sin(yaw);

      // Euler angles, right-handed, intrinsic, XYZ convention
      // (which means: rotate around body axes Z, Y', X'') 
      m[0][0] = cp * cy;
      m[0][1] = cy * sr * sp - cr * sy;
      m[0][2] = sr * sy + cr * cy * sp;

      m[1][0] = cp * sy;
      m[1][1] = cr * cy + sr * sp * sy;
      m[1][2] = cr * sp * sy - cy * sr;

      m[2][0] = -sp;
      m[2][1] = cp * sr;
      m[2][2] = cr * cp;
    }

    // Read every sensor and record a time stamp
    // Init DCM with unfiltered orientation
    // TODO re-init global vars?
    function reset_sensor_fusion(accel, MAG_Heading) {
      var temp1 = [];
      var temp2 = [];
      var xAxis = [1.0, 0.0, 0.0];

      // Euler angles
      var yaw = 0.0;
      var pitch = 0.0;
      var roll = 0.0;

      //read_sensors();
      timestamp = Date.now();
      
      // GET PITCH
      // Using y-z-plane-component/x-component of gravity vector
      pitch = -Math.atan2(accel[0], Math.sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
        
      // GET ROLL
      // Compensate pitch of gravity vector 
      temp1 = Vector_Cross_Product(accel, xAxis);
      temp2 = Vector_Cross_Product(xAxis, temp1);
      
      // Normally using x-z-plane-component/y-component of compensated gravity vector
      // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
      // Since we compensated for pitch, x-z-plane-component equals z-component:
      roll = Math.atan2(temp2[1], temp2[2]);

      // GET YAW
      //Compass_Heading();  //NEED TO SUBSTITUTE ICOMPASS
      yaw = MAG_Heading;  
        
      // Init rotation matrix
      init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);

    }

    function constrain(n, min, max) {
      return Math.max(min, Math.min(n, max));
    };

    //------------- DCM.INO -------------------------

    // DCM algorithm

    /**************************************************/
    function Normalize(){
      var error=0;
      var temporary = [[], [], []];
      var renorm = 0;
      
      error = -Vector_Dot_Product(DCM_Matrix[0], DCM_Matrix[1])*0.5; //eq.19

      temporary[0] = Vector_Scale(DCM_Matrix[1], error); //eq.19
      temporary[1] = Vector_Scale(DCM_Matrix[0], error); //eq.19
      
      temporary[0] = Vector_Add(temporary[0], DCM_Matrix[0]);//eq.19
      temporary[1] = Vector_Add(temporary[1], DCM_Matrix[1]);//eq.19
      
      temporary[2] = Vector_Cross_Product(temporary[0], temporary[1]); // c= a x b //eq.20
      
      renorm = 0.5 *(3 - Vector_Dot_Product(temporary[0], temporary[0])); //eq.21
      DCM_Matrix[0] = Vector_Scale(temporary[0], renorm);
      
      renorm = 0.5 *(3 - Vector_Dot_Product(temporary[1], temporary[1])); //eq.21
      DCM_Matrix[1] = Vector_Scale(temporary[1], renorm);
      
      renorm = 0.5 *(3 - Vector_Dot_Product(temporary[2], temporary[2])); //eq.21
      DCM_Matrix[2] = Vector_Scale(temporary[2], renorm);
    }

    /**************************************************/
    function Drift_correction(gyro, accel, MAG_Heading){
      var mag_heading_x;
      var mag_heading_y;
      var errorCourse;

      //Compensation the Roll, Pitch and Yaw drift. 
      var Scaled_Omega_P = [];
      var Scaled_Omega_I = [];
      var Accel_magnitude;
      var Accel_weight;
      var Accel_Vector = [];
      
      //*****Roll and Pitch***************
      // Calculate the magnitude of the accelerometer vector

      // Store the acceleration in a vector
      Accel_Vector = accel;
      Accel_magnitude = Math.sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
      Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.

      // Dynamic weighting of accelerometer info (reliability filter)
      // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
      Accel_weight = constrain(1 - 2*Math.abs(1 - Accel_magnitude),0,1);  //  

      var errorRollPitch = [0, 0, 0];
      errorRollPitch = Vector_Cross_Product(Accel_Vector, DCM_Matrix[2]); //adjust the ground of reference
      Omega_P = Vector_Scale(errorRollPitch, Kp_ROLLPITCH*Accel_weight);
      
      Scaled_Omega_I = Vector_Scale(errorRollPitch, Ki_ROLLPITCH*Accel_weight);
      Omega_I = Vector_Add(Omega_I, Scaled_Omega_I);     
      
      //*****YAW***************
      // We make the gyro YAW drift correction based on compass magnetic heading
      var errorYaw = [0, 0, 0];
      
      mag_heading_x = Math.cos(MAG_Heading);
      mag_heading_y = Math.sin(MAG_Heading);
      errorCourse = (DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
      errorYaw = Vector_Scale(DCM_Matrix[2], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
      
      Scaled_Omega_P = Vector_Scale(errorYaw, Kp_YAW);//.01proportional of YAW.
      Omega_P = Vector_Add(Omega_P, Scaled_Omega_P);//Adding  Proportional.
      
      Scaled_Omega_I = Vector_Scale(errorYaw, Ki_YAW);//.00001Integrator
      Omega_I = Vector_Add(Omega_I, Scaled_Omega_I);//adding integrator to the Omega_I
    }

    // @param gyro [x roll, y pitch, z yaw]
    // @param accel
    // @param G_Dt delta t
    function Matrix_update(gyro, accel, G_Dt){
      var Update_Matrix = [[0, 1, 2], [3, 4, 5], [6, 7, 8]];
      var Omega_Vector = [0, 0, 0]; // Corrected Gyro_Vector data
      var Omega = [0, 0, 0];

      var Gyro_vector = gyro; //radians

      Omega = Vector_Add(Gyro_vector, Omega_I);  //adding proportional term
      Omega_Vector = Vector_Add(Omega, Omega_P); //adding Integrator term
      
      // Use drift correction
      Update_Matrix[0][0] = 0;
      Update_Matrix[0][1] = -G_Dt*Omega_Vector[2];//-z
      Update_Matrix[0][2] = G_Dt*Omega_Vector[1];//y
      Update_Matrix[1][0] = G_Dt*Omega_Vector[2];//z
      Update_Matrix[1][1] = 0;
      Update_Matrix[1][2] = -G_Dt*Omega_Vector[0];//-x
      Update_Matrix[2][0] = -G_Dt*Omega_Vector[1];//-y
      Update_Matrix[2][1] = G_Dt*Omega_Vector[0];//x
      Update_Matrix[2][2] = 0;
      
      var Temporary_Matrix = Matrix_Multiply(DCM_Matrix, Update_Matrix); //a*b=c

      for(var x=0; x<3; x++){ //Matrix Addition (update)
        for(var y=0; y<3; y++){
          DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
      }
    }

    function getEulerRad(){
      var angles = [];
      angles[1] = Math.asin(DCM_Matrix[2][0]);  //originally -asin
      angles[2] = Math.atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
      angles[0] = Math.atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
      return angles;
    }

    function getEulerDeg(){
      var angles = [];
      angles[1] = TO_DEG(Math.asin(DCM_Matrix[2][0]));
      angles[2] = TO_DEG(Math.atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]));
      angles[0] = TO_DEG(Math.atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]));
      return angles;
    }

    //
    //DCM to Quaternion conversion based on Vector Nav Application Note
    //AN002, Quaternion Math
    // 
    function getDCM2Q(){
      //quaternions as defined in Vector Nav App Note AN002
      var q3 = 0.5 * Math.sqrt(DCM_Matrix[0][0] + DCM_Matrix[1][1] + DCM_Matrix[2][2] + 1);
      var q0 = (DCM_Matrix[1][2]-DCM_Matrix[2][1])/(4*q3);
      var q1 = (DCM_Matrix[2][0]-DCM_Matrix[0][2])/(4*q3);
      var q2 = (DCM_Matrix[0][1]-DCM_Matrix[1][0])/(4*q3);

      //convert to quaternion notation as used in the FreeIMU library
      return {
        w: q3,
        x: -q0,
        y: -q1,
        z: -q2
      };
    }

    function calDCM(gx, gy, gz, ax, ay, az, heading, deltaTimeSec) {
        heading = TO_RAD(heading);

        if (DCM_Matrix == null){
          DCM_Matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
          reset_sensor_fusion([ax, ay, az], heading);
        }

        Matrix_update([gx, gy, gz], [ax, ay, az], deltaTimeSec);
        Normalize();
        Drift_correction([gx, gy, gz], [ax, ay, az], heading);
        
        timestamp_old = timestamp;
        timestamp = Date.now();
    }

    return {
        update: calDCM,
        getQuaternion: getDCM2Q
    };

    //====================================================================================================
    // END OF CODE
    //====================================================================================================

};

},{}],"./Madgwick":[function(require,module,exports){
//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
// 19/02/2012   SOH Madgwick    Magnetometer measurement is normalised
//
//=====================================================================================================

'use strict';

/**
 * The Madgwick algorithm.  See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * @param {number} sampleInterval The sample interval in milliseconds.
 */
module.exports = function Madgwick(sampleInterval, options) {

    //---------------------------------------------------------------------------------------------------
    // Definitions

    options = options || {};
    var sampleFreq = 1000 / sampleInterval;  // sample frequency in Hz
    var beta = options.beta || 1.0;   // 2 * proportional gain - lower numbers are smoother, but take longer to get to correct attitude.

    //---------------------------------------------------------------------------------------------------
    // Variable definitions
    var q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
    var recipSampleFreq = 1.0 / sampleFreq;

    return {
        update: madgwickAHRSupdate,
        getQuaternion: function() {
            return {
                w: q0,
                x: q1,
                y: q2,
                z: q3
            };
        }
    };


    //====================================================================================================
    // Functions

    //---------------------------------------------------------------------------------------------------
    // IMU algorithm update
    function madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az) {
        var recipNorm;
        var s0, s1, s2, s3;
        var qDot1, qDot2, qDot3, qDot4;
        var V_2q0, V_2q1, V_2q2, V_2q3, V_4q0, V_4q1, V_4q2, V_8q1, V_8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax === 0.0) && (ay === 0.0) && (az === 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = Math.pow(ax * ax + ay * ay + az * az, -0.5);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            V_2q0 = 2.0 * q0;
            V_2q1 = 2.0 * q1;
            V_2q2 = 2.0 * q2;
            V_2q3 = 2.0 * q3;
            V_4q0 = 4.0 * q0;
            V_4q1 = 4.0 * q1;
            V_4q2 = 4.0 * q2;
            V_8q1 = 8.0 * q1;
            V_8q2 = 8.0 * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = V_4q0 * q2q2 + V_2q2 * ax + V_4q0 * q1q1 - V_2q1 * ay;
            s1 = V_4q1 * q3q3 - V_2q3 * ax + 4.0 * q0q0 * q1 - V_2q0 * ay - V_4q1 + V_8q1 * q1q1 + V_8q1 * q2q2 + V_4q1 * az;
            s2 = 4.0 * q0q0 * q2 + V_2q0 * ax + V_4q2 * q3q3 - V_2q3 * ay - V_4q2 + V_8q2 * q1q1 + V_8q2 * q2q2 + V_4q2 * az;
            s3 = 4.0 * q1q1 * q3 - V_2q1 * ax + 4.0 * q2q2 * q3 - V_2q2 * ay;
            recipNorm = Math.pow(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3, -0.5); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * recipSampleFreq;
        q1 += qDot2 * recipSampleFreq;
        q2 += qDot3 * recipSampleFreq;
        q3 += qDot4 * recipSampleFreq;

        // Normalise quaternion
        recipNorm = Math.pow(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, -0.5);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    //---------------------------------------------------------------------------------------------------
    // AHRS algorithm update

    function madgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltaTimeSec) {
        recipSampleFreq = deltaTimeSec || recipSampleFreq;
        var recipNorm;
        var s0, s1, s2, s3;
        var qDot1, qDot2, qDot3, qDot4;
        var hx, hy;
        var V_2q0mx, V_2q0my, V_2q0mz, V_2q1mx, V_2bx, V_2bz, V_4bx, V_4bz, V_2q0, V_2q1, V_2q2, V_2q3, V_2q0q2, V_2q2q3;
        var q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if ((mx==0.0) && (my == 0.0) && (mz == 0.0)) {
            madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = Math.pow(ax * ax + ay * ay + az * az, -0.5);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = Math.pow(mx * mx + my * my + mz * mz, -0.5);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            V_2q0mx = 2.0 * q0 * mx;
            V_2q0my = 2.0 * q0 * my;
            V_2q0mz = 2.0 * q0 * mz;
            V_2q1mx = 2.0 * q1 * mx;
            V_2q0 = 2.0 * q0;
            V_2q1 = 2.0 * q1;
            V_2q2 = 2.0 * q2;
            V_2q3 = 2.0 * q3;
            V_2q0q2 = 2.0 * q0 * q2;
            V_2q2q3 = 2.0 * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - V_2q0my * q3 + V_2q0mz * q2 + mx * q1q1 + V_2q1 * my * q2 + V_2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = V_2q0mx * q3 + my * q0q0 - V_2q0mz * q1 + V_2q1mx * q2 - my * q1q1 + my * q2q2 + V_2q2 * mz * q3 - my * q3q3;
            V_2bx = Math.sqrt(hx * hx + hy * hy);
            V_2bz = -V_2q0mx * q2 + V_2q0my * q1 + mz * q0q0 + V_2q1mx * q3 - mz * q1q1 + V_2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            V_4bx = 2.0 * V_2bx;
            V_4bz = 2.0 * V_2bz;

            // Gradient decent algorithm corrective step
            s0 = -V_2q2 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q1 * (2.0 * q0q1 + V_2q2q3 - ay) - V_2bz * q2 * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (-V_2bx * q3 + V_2bz * q1) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + V_2bx * q2 * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s1 = V_2q3 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q0 * (2.0 * q0q1 + V_2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + V_2bz * q3 * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (V_2bx * q2 + V_2bz * q0) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + (V_2bx * q3 - V_4bz * q1) * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s2 = -V_2q0 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q3 * (2.0 * q0q1 + V_2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-V_4bx * q2 - V_2bz * q0) * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (V_2bx * q1 + V_2bz * q3) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + (V_2bx * q0 - V_4bz * q2) * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s3 = V_2q1 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q2 * (2.0 * q0q1 + V_2q2q3 - ay) + (-V_4bx * q3 + V_2bz * q1) * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (-V_2bx * q0 + V_2bz * q2) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + V_2bx * q1 * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            recipNorm = Math.pow(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3, -0.5); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * recipSampleFreq;
        q1 += qDot2 * recipSampleFreq;
        q2 += qDot3 * recipSampleFreq;
        q3 += qDot4 * recipSampleFreq;

        // Normalise quaternion
        recipNorm = Math.pow(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, -0.5);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    //====================================================================================================
    // END OF CODE
    //====================================================================================================

};

},{}],"./Mahony":[function(require,module,exports){
//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=====================================================================================================

'use strict';

/**
 * The Mahony algorithm.  See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * @param {number} sampleInterval The sample interval in milliseconds.
 */
module.exports = function Mahony(sampleInterval, options) {

    //---------------------------------------------------------------------------------------------------
    // Definitions

    options = options || {};
    var kp = options.kp || 1.0;
    var ki = options.ki || 0.0;

    var sampleFreq = 1000 / sampleInterval;  // sample frequency in Hz
    var twoKpDef = 2.0 * kp;                 // 2 * proportional gain
    var twoKiDef = 2.0 * ki;                 // 2 * integral gain
    var recipSampleFreq = 1 / sampleFreq;

    //---------------------------------------------------------------------------------------------------
    // Variable definitions

    var twoKp = twoKpDef;                                                       // 2 * proportional gain (Kp)
    var twoKi = twoKiDef;                                                       // 2 * integral gain (Ki)
    var q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;                                 // quaternion of sensor frame relative to auxiliary frame
    var integralFBx = 0.0, integralFBy = 0.0, integralFBz = 0.0;                // integral error terms scaled by Ki


    return {
        update: mahonyAHRSupdate,
        getQuaternion: function() {
            return {
                w: q0,
                x: q1,
                y: q2,
                z: q3
            };
        }
    };

    //====================================================================================================
    // Functions

    //---------------------------------------------------------------------------------------------------
    // IMU algorithm update
    //

    function mahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az) {
        var recipNorm;
        var halfvx, halfvy, halfvz;
        var halfex, halfey, halfez;
        var qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax === 0.0) && (ay === 0.0) && (az === 0.0))) {
            // Normalise accelerometer measurement
            // 2.
            recipNorm = Math.pow(ax * ax + ay * ay + az * az, -0.5);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            // 3. get estimated gravity vector d/2 (halfv) from quaternion q
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5 + q3 * q3;

            // 4. Calculate error vector e/2 (halfe)
            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute integral feedback if enabled
            // 5. calculate I term (integralFB)
            if (twoKi > 0.0) {
                integralFBx += twoKi * halfex * recipSampleFreq; // integral error scaled by Ki
                integralFBy += twoKi * halfey * recipSampleFreq;
                integralFBz += twoKi * halfez * recipSampleFreq;
            } else {
                integralFBx = 0.0; // prevent integral windup
                integralFBy = 0.0;
                integralFBz = 0.0;
            }
            // Apply proportional feedback
            // 6. calculate P term and add everything together
            gx += twoKp * halfex + integralFBx;
            gy += twoKp * halfey + integralFBy;
            gz += twoKp * halfez + integralFBz;
        }

        // Integrate rate of change of quaternion
        // 7.
        gx *= (0.5 * recipSampleFreq);         // pre-multiply common factors
        gy *= (0.5 * recipSampleFreq);
        gz *= (0.5 * recipSampleFreq);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        // 8.
        recipNorm = Math.pow(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, -0.5);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    //
    //---------------------------------------------------------------------------------------------------
    // AHRS algorithm update
    //

    function mahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltaTimeSec) {
        recipSampleFreq = deltaTimeSec || recipSampleFreq;
        var recipNorm;
        var q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        var hx, hy, hz, bx, bz;
        var halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        var halfex, halfey, halfez;
        var qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if ((mx==0.0) && (my == 0.0) && (mz == 0.0)) {
            mahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            // 2.
            recipNorm = Math.pow(ax * ax + ay * ay + az * az, -0.5);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = Math.pow(mx * mx + my * my + mz * mz, -0.5);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            // compute flux in the earth frame
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            hz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));
            // normalise the flux vector to have only components in the x and z
            bx = Math.sqrt(hx * hx + hy * hy);
            bz = hz;

            // Estimated direction of gravity and magnetic field
            // 3. get estimated gravity vector d/2 (halfv) from quaternion 
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5 + q3q3;
            
            // 3.1. get estimated magnetic field c/2 (halfw) from quaternion 
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            // 4. calculate error vector e/2 (halfe) e = a x d + m x c
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute integral feedback if enabled
            // 5. calculate I term (integralFB)
            if (twoKi > 0.0) {
                integralFBx += twoKi * halfex * recipSampleFreq;  // integral error scaled by Ki
                integralFBy += twoKi * halfey * recipSampleFreq;
                integralFBz += twoKi * halfez * recipSampleFreq;
            } else {
                integralFBx = 0.0;  // prevent integral windup
                integralFBy = 0.0;
                integralFBz = 0.0;
            }
            
            // Apply proportional feedback
            // 6. calculate P term and add everything together
            gx += twoKp * halfex + integralFBx;
            gy += twoKp * halfey + integralFBy;
            gz += twoKp * halfez + integralFBz;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * recipSampleFreq);    // pre-multiply common factors
        gy *= (0.5 * recipSampleFreq);
        gz *= (0.5 * recipSampleFreq);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        // 8.
        recipNorm = Math.pow(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, -0.5);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    //====================================================================================================
    // END OF CODE
    //====================================================================================================

};

},{}],"ahrs":[function(require,module,exports){
/*********************************************************************
 *                                                                   *
 *   Copyright 2016 Simon M. Werner                                  *
 *                                                                   *
 *   Licensed to the Apache Software Foundation (ASF) under one      *
 *   or more contributor license agreements.  See the NOTICE file    *
 *   distributed with this work for additional information           *
 *   regarding copyright ownership.  The ASF licenses this file      *
 *   to you under the Apache License, Version 2.0 (the               *
 *   "License"); you may not use this file except in compliance      *
 *   with the License.  You may obtain a copy of the License at      *
 *                                                                   *
 *      http://www.apache.org/licenses/LICENSE-2.0                   *
 *                                                                   *
 *   Unless required by applicable law or agreed to in writing,      *
 *   software distributed under the License is distributed on an     *
 *   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY          *
 *   KIND, either express or implied.  See the License for the       *
 *   specific language governing permissions and limitations         *
 *   under the License.                                              *
 *                                                                   *
 *********************************************************************/

'use strict';

function AHRS(options) {
    options = options || {};
    var sampleInterval = options.sampleInterval || 20;
    var algorithmName = options.algorithm || 'Madgwick';

    var algorithmFn;
    if (algorithmName === 'Mahony') {
        algorithmFn = new (require('./Mahony'))(sampleInterval, options);
    } else if (algorithmName === 'Madgwick') {
        algorithmFn = new (require('./Madgwick'))(sampleInterval, options);
    } else if (algorithmName === 'DCM') {
        algorithmFn = new (require('./DCM'))(sampleInterval, options);
    } else {
        throw new Error('AHRS(): Algorithm not valid: ', algorithmName);
    }

    // Copy all properties accross
    for (var prop in algorithmFn) {
        if (algorithmFn.hasOwnProperty(prop)) {
            this[prop] = algorithmFn[prop];
        }
    }

}

/**
 * Convert the quaternion to a vector with angle.  Reverse of the code
 * in the following link: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
 * @return {object} Normalised vector - {x, y, z, angle}
 */
AHRS.prototype.toVector = function () {
    var q = this.getQuaternion();
    var angle = 2 * Math.acos(q.w);
    var sinAngle = Math.sin(angle / 2);
    return {
        angle: angle,
        x: q.x / sinAngle,
        y: q.y / sinAngle,
        z: q.z / sinAngle
    };
};

/**
 * Return an object with the Euler angles {heading, pitch, roll}, in radians.
 *
 * Where:
 *   - heading is from magnetic north, going west (about z-axis).
 *   - pitch is from vertical, going forward (about y-axis).
 *   - roll is from vertical, going right (about x-axis).
 *
 * Thanks to:
 *   https://github.com/PenguPilot/PenguPilot/blob/master/autopilot/service/util/math/quat.c#L103
 * @return {object} {heading, pitch, roll} in radians
 */
AHRS.prototype.getEulerAngles = function() {
    if (typeof this.getEulerRad == 'function'){
        return this.getEulerRad();
    }
    var q = this.getQuaternion();
    var ww = q.w * q.w, xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    return {
        heading: Math.atan2(2 * (q.x * q.y + q.z * q.w), xx - yy - zz + ww),
        pitch: -Math.asin(2 * (q.x * q.z - q.y * q.w)),
        roll: Math.atan2(2 * (q.y * q.z + q.x * q.w), -xx - yy + zz + ww)
    };
};

module.exports = AHRS;

},{"./DCM":"./DCM","./Madgwick":"./Madgwick","./Mahony":"./Mahony"}]},{},[]);
