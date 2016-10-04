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
    var GRAVITY = 1.0;
    
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
      var angles = {
          pitch: Math.asin(DCM_Matrix[2][0]),  //originally -asin
          roll: Math.atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]),
          heading: Math.atan2(DCM_Matrix[1][0], DCM_Matrix[0][0])
      }
      return angles;
    }

    function getEulerDeg(){
      var angles = {
          pitch: TO_DEG(Math.asin(DCM_Matrix[2][0])),
          roll: TO_DEG(Math.atan2(DCM_Matrix[2][1], DCM_Matrix[2][2])),
          heading: TO_DEG(Math.atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]))
      }
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
        getQuaternion: getDCM2Q,
        getEulerRad: getEulerRad,
        getEulerDeg: getEulerDeg
    };

    //====================================================================================================
    // END OF CODE
    //====================================================================================================

};
