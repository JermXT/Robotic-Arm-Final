

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;
using System;
using System.Threading;

public class Cleaner : MonoBehaviour {
    //SerialPort ARM = new SerialPort("COM5", 115200);     // Change Port
    public SerialPort ARM = new SerialPort("/dev/tty.usbmodem14101", 115200);     // Change Port
    //SerialPort ARM;
    private static Cleaner inst;
    public static Cleaner instance { get { return inst; } }

    private void Awake()
    {
        if (inst != null && inst != this)
            Destroy(this.gameObject);
        else
            inst = this;
    }

    /// <summary>
    /// Gets or sets the sample period.
    /// </summary>
    [SerializeField] public float SamplePeriod = 512;

    /// <summary>
    /// State, x, y, z, d/dx, d/dy, d/dz
    /// </summary>
    [SerializeField] public float[] state = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    [SerializeField] public float[] calibrateValues = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    [SerializeField] public float calibCount = 0.0f;
    //[SerializeField] public float[][] roll = {{0.0f},{0.0f};   

    //[SerializeField] public float[] P = 
    //[SerializeField] public float[] F =
    //[SerializeField] public float[] H =
    //[SerializeField] public float[] R =
    //[SerializeField] public float[] Q =

    /// <summary>
    /// Motor state x,y,z
    /// </summary>
    [SerializeField] public int[] motorState = {0,0,0};

    /// <summary>
    /// Gets or sets the algorithm gain beta.
    /// </summary>
    [SerializeField] public float Beta = 0.1f;

    /// <summary>
    /// Gets or sets the Quaternion output.
    /// </summary>
    [SerializeField] public float[] quat;
    [SerializeField] public float beginTime = 0.0f;
    [SerializeField] public float bTime = 0.0f;
    [SerializeField] public float totalTime = 0;
    [SerializeField] public float motorTime = 0;
    /// <summary>
    /// Matrix state
    /// </summary>
    [SerializeField] public Matrix rotationState = new Matrix(6,1);
    //roll pitch yaw droll dpitch pyaw
    [SerializeField] public Matrix rotationCovar = new Matrix(6,6);
    [SerializeField] public Matrix Fr = new Matrix(6,6);
    [SerializeField] public Matrix Hr = new Matrix(3,6);
    [SerializeField] public Matrix Rr = new Matrix(3,3);
    [SerializeField] public Matrix Qr = new Matrix(6,6);

    [SerializeField] public Matrix positionState = new Matrix(9,1);
    [SerializeField] public Matrix positionCovar = new Matrix(9,9);
    [SerializeField] public Matrix Fp = new Matrix(15,15);
    [SerializeField] public Matrix Hp = new Matrix(9,15);
    [SerializeField] public Matrix Rp = new Matrix(9,9);
    [SerializeField] public Matrix Qp = new Matrix(9,9);
    public int test =0;
    

    public bool transition= false;
    void Start () {
        ARM.ReadTimeout = 50;
        rotationCovar.mat[0][0] = 1000000;
        rotationCovar.mat[1][1] = 1000000;
        rotationCovar.mat[2][2] = 1000000;
        rotationCovar.mat[3][3] = 90;
        rotationCovar.mat[4][4] = 90;
        rotationCovar.mat[5][5] = 90;
        

        Fr.mat[0][0] = 1.0f;
        Fr.mat[1][1] = 1.0f;
        Fr.mat[2][2] = 1.0f;
        Fr.mat[3][3] = 1.0f;
        Fr.mat[4][4] = 1.0f;
        Fr.mat[5][5] = 1.0f;

        Hr.mat[0][3] = 1;
        Hr.mat[1][4] = 1;
        Hr.mat[2][5] = 1;

        Rr.mat[0][0] =1.0f;
        Rr.mat[1][1] = 1.0f;
        Rr.mat[2][2] = 1.0f;

        //Rr.mat[0][0] =0.0093377964f;
        //Rr.mat[1][1] = 0.0109793324f;
        //Rr.mat[2][2] = 0.0122546351f;
        //Rr.mat[3][3] =1.0f;
        //Rr.mat[4][4] = 1.0f;
        //Rr.mat[5][5] = 1.0f;



/*
        F.mat[0][0] = 1f;
        F.mat[0][3] = 0.0f;
        F.mat[1][1] = 1f;
        F.mat[1][4] = 0.0f;
        F.mat[2][2] = 1f;
        F.mat[2][5] = 0.0f;
        F.mat[3][3] = 1f;
        F.mat[4][4] = 1f;
        F.mat[5][5] = 1f;
        F.mat[6][6] = 1f;
        F.mat[7][7] = 1f;
        F.mat[8][8] = 1f;
        F.mat[9][9] = 1f;
        F.mat[9][12] = 0.0f;
        F.mat[10][10] = 1f;
        F.mat[10][13] = 0.0f;
        F.mat[11][11] = 1f;
        F.mat[11][14] = 0.0f;
        F.mat[12][12] = 1f;
        F.mat[13][13] = 1f;
        F.mat[14][14] = 1f;
        */
        //8,11
        //7,10
        //6,9
        //8,14
        //7,13
        //6,12
        //Debug.Log(F.ToString());

        // gyro,angle, accel

 //       H.mat[0][3] = 1;
 //       H.mat[1][4] = 1;
 //       H.mat[2][5] = 1;
 //       H.mat[3][0] = 1;
//        H.mat[4][1] = 1;
//        H.mat[5][2] = 1;
//        H.mat[6][12] = 1;
//        H.mat[7][13] = 1;
//        H.mat[8][14] = 1;
        /*
        H.mat[0][3] = 1;
        H.mat[1][4] = 1;
        H.mat[2][5] = 1;
        H.mat[]
        */
        //R.mat[0][0] = 0.484476789255f;
        //R//.mat[1][1] = 0.586474759512f;
        //R.mat[2][2] = 0.624911487605f;
//        R.mat[0][0] =0.0093377964f;
//        R.mat[1][1] = 0.0109793324f;
//        R.mat[2][2] = 0.0122546351f;
        //change belwo for angle l8r
        //R.mat[3][3] =50000000000000f;
        //R.mat[4][4] =50f;
        //R.mat[5][5] =50f;
/*
        R.mat[3][3] =5000000000000000000f;
        R.mat[4][4] =5000000000000000000f;
        R.mat[5][5] =5000000000000000000f;
        R.mat[6][6] = 0.0000077f;
        R.mat[7][7] = 0.000009963f;
        R.mat[8][8] = 0.0000243f;
  */      
        /*
        R.mat[0][0] = 50000000;
        R.mat[1][1] = 50000000;
        R.mat[2][2] = 50000000;
        */
        /*
        Qr.mat[0][0] = 0.000025f;
        Qr.mat[0][1] = 0.000005f;
        Qr.mat[1][0] = 0.000005f;
        Qr.mat[1][1] = 0.01f;
        
        Qr.mat[2][2] = 0.000025f;
        Qr.mat[2][3] = 0.000005f;
        Qr.mat[3][2] = 0.000005f;
        Qr.mat[3][3] = 0.01f;
        Qr.mat[4][4] = 0.000025f;
        Qr.mat[4][5] = 0.000005f;
        Qr.mat[5][4] = 0.000005f;
        Qr.mat[5][5] = 0.01f;*/
        /*
        rotationState.mat[0][0] = 1f;
        rotationState.mat[1][0] = 2f;
        rotationState.mat[2][0] = 3f;
        rotationState.mat[3][0] = 30f;
        rotationState.mat[4][0] = 20f;
        rotationState.mat[5][0] = 10f;
        */
        //Debug.Log(rotationState.rows);
        //Debug.Log(rotationState.cols);
        //Debug.Log(F.cols);
        //Debug.Log(F.rows);
        //Debug.Log(F.ToString());
        //Debug.Log(rotationState.ToString());

        //Debug.Log(res.ToString());


        //Debug.Log(rotationState.ToString());
        //Debug.Log(rotationCovar.ToString());
        //Debug.Log(Hr.ToString());
        //Debug.Log(Fr.ToString());
        //Debug.Log(Rr.ToString());
        
        //ARM.ReadTimeout = 1;
    }
    public float errorPitch;
    public float errorRoll;

    public void kalmanRotationImu(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    { 

        //gx-=50.1443210931f;
        //gy+=9.57899231426f;
        //gz-=7.20409906063f;
        //Debug.Log(gx + " " + gy + " " + gz);
        totalTime+=dt;
        motorTime+=dt;
        gx-=calibrateValues[0]/calibCount;
        gy-=calibrateValues[1]/calibCount;
        gz-=calibrateValues[2]/calibCount;
        //Debug.Log(gz);
        //Debug.Log(calibrateValues[2]);
        //if(bTime > 1.0){
        //    float place = dt;
        //    dt = dt-bTime;
        //    bTime = place;
        //} else {
        //    bTime+=dt;
        //}
        Fr.mat[0][3] = dt;
        Fr.mat[1][4] = dt;
        Fr.mat[2][5] = dt;
        //Debug.Log(Fr.ToString());
        //Debug.Log(Fr);
        /*
        F.mat[0][3] = dt;
        F.mat[1][4] = dt;
        F.mat[2][5] = dt;
        F.mat[9][12] = dt;
        F.mat[10][13] = dt;
        F.mat[11][14] = dt;
        F.mat[8][11]= dt;
        F.mat[7][10]= dt;
        F.mat[6][9]= dt;
        F.mat[8][14]= dt*dt/2;
        F.mat[7][13]= dt*dt/2;
        F.mat[6][12]= dt*dt/2;
        */
        //Debug.Log(F.ToString());
        //Predict
        rotationState = Fr*rotationState;

        //Debug.Log(rotationState.ToString());
        
        rotationCovar = Fr*rotationCovar*Fr.T+Qr;
        //Debug.Log(1);
        //Debug.Log(rotationCovar.ToString());
        //Update
        Matrix S = Hr*rotationCovar*Hr.T+Rr;
        //Debug.Log(S.ToString());
        Matrix K = rotationCovar*Hr.T*S.Inverse();
        //Debug.Log(rotationCovar.ToString());
        //Debug.Log(K.ToString());
        Matrix zState = Hr*rotationState;
        //Debug.Log("wait");
        //Debug.Log(zState.ToString());
        //AccelMagAngles.iecompass((Int16)(gx*131f),(Int16)(gy*131f),(Int16)(gz*131f),(Int16)(ax*16384f),(Int16)(ay*16384f),(Int16)(az*16384f));
//        Matrix z = new Matrix(9,1);
        Matrix z = new Matrix(3,1);      
        
        z.mat[0][0] = gx;
        z.mat[1][0] = gy;
        z.mat[2][0] = gz;
        //Debug.Log(z.mat[0][0]+" "+z.mat[1][0]+" "+z.mat[2][0]);
        //z.mat[3][0] = AccelMagAngles.iPhi;
        //z.mat[4][0] = AccelMagAngles.iThe;
        //z.mat[5][0] = AccelMagAngles.iPsi;
        //double roll = Mathf.Atan((ay)/Mathf.Sqrt(Mathf.Pow(ax,2) + Mathf.Pow(az,2)))*180/3.141592654;
        //double pitch = Mathf.Atan(-1*ax/Mathf.Sqrt(Mathf.Pow(ay,2) + Mathf.Pow(az,2)))*180/3.141592654; 
        
//z.mat[5][0] = (float)roll;
//z.mat[4][0] = (float)pitch;
//z.mat[6][0] = ax;
//z.mat[7][0] = ay;
//z.mat[8][0] = az;
        
     //   Debug.Log(z.mat[3][0]+" "+z.mat[4][0]+" "+z.mat[5][0]);
        //Debug.Log(z.ToString());
        //Debug.Log(zState.ToString());

        Matrix y = z-zState;

        //Debug.Log(y.ToString());
        //Debug.Log(K.ToString());
        //Debug.Log("amount");
        //Debug.Log(K*y);
        //Debug.Log(K*y);
        
        rotationState = rotationState + K*y;
        //Debug.Log(K*y);

        //Debug.Log(rotationCovar.ToString());

        rotationCovar = rotationCovar - K*Hr*rotationCovar;
        //Debug.Log(2);
        //Debug.Log(rotationCovar.ToString());
        //Debug.Log(rotationCovar.ToString());
        //Debug.Log("sts");
        //Debug.Log(rotationState.ToString());

        //System.String toMotorX = ((rotationState.mat[0][0])%360).ToString();
        //System.String toMotorY = ((rotationState.mat[1][0])%360).ToString();
        //System.String toMotorZ = ((rotationState.mat[2][0])%360).ToString();
       
        //red
        //Quaternion rotation = Quaternion.Euler(rotationState.mat[2][0]%360, rotationState.mat[0][0]%360, rotationState.mat[1][0]%360);
        Debug.Log(rotationState.ToString());
        Debug.Log(K*y);
        Debug.Log(z.ToString());
        Debug.Log("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        //blue
        Quaternion rotation = Quaternion.Euler(-rotationState.mat[1][0]%360, rotationState.mat[0][0]%360, rotationState.mat[2][0]%360);

        //Debug.Log(rotationState.mat[0][0]);
        quat[0] = rotation.w;
        quat[1] = rotation.x;
        quat[2] = rotation.y;
        quat[3] = rotation.z;

        //Debug.Log(toMotorX);
        //Debug.Log(toMotorX);
        //Debug.Log(toMotorX);
        //Debug.Log("=-----------");

        //if(totalTime%1000>800){
        //StringToArduino(toMotorX);
        //StringToArduino(toMotorY);
        //StringToArduino(toMotorZ);
        //}
        //char a = 'X';
        //char b = '1';
        //char c = 'Y';
        //char d = '1';

        //byte[] test = {(byte)a,(byte)b,(byte)c,(byte)d};
        //WriteToArduino(test,0,4);
        //char a = 'W';
        
        
        //byte[] test = {(byte)a};
        //WriteToArduino(test,0,1);
        //Debug.Log(test);
        //test +=1;
        //ThreadStart childref = new ThreadStart(StringToArduino);
        int zStep =0;
        //Debug.Log(rotationState.mat[0][0]+" "+rotationState.mat[1][0]+" "+rotationState.mat[2][0]);
        if(rotationState.mat[0][0]%360>0){
            zStep = (int)Math.Round((rotationState.mat[0][0]%360)/360.0f*400.0f);
        }else if(rotationState.mat[0][0]%360<0){
            zStep = (int)Math.Round(-(rotationState.mat[0][0]%360)/360.0f*400.0f)+200;
        }

        
        String tempNum =zStep+"";
        String zStepPos = "";
        for(int i =0; i < 3-tempNum.Length;i++){
            zStepPos += "0";
        }
        zStepPos += zStep;
        if(motorTime>0.5){
            //Thread thread = new Thread(() => StringToArduino("000"+zStepPos+"000F"));
            Thread thread = new Thread(() => StringToArduino(zStepPos+"000000F"));
            //Thread thread = new Thread(() => StringToArduino(zStepPos));
            thread.Start();
        //    Debug.Log("000"+zStepPos+"000F");
            motorTime-=0.5f;
            //StringToArduino("000000050");
        }

        //char b = '3';
        //byte[] test1 = {(byte)b};
        //WriteToArduino(test1,0,1);
        
        

    }

/////////////////////////////////////////////////
    public Quaternion GetQuaternionData(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        //Debug.Log("GetQuaternionData "+ gx);
        /*
        if(beginTime<5.0f){
            calibrate(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        } else {
            if(transition == false){
                transition = true;
                state[3]= -calibrateValues[3];
                state[4]= -calibrateValues[4];
                state[5]= -calibrateValues[5];
                calibrateValues[0]= -calibrateValues[0]/dt;
                calibrateValues[2]= -calibrateValues[1]/dt;
                calibrateValues[1]= -calibrateValues[2]/dt;
            }
            UpdateImu(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        }*/
        
        if(calibCount < 50){
            calibrate(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
            calibCount++;
            return new Quaternion(quat[0], quat[1], quat[2], quat[3]);
        }
     
        //UpdateImu(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        kalmanRotationImu(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        Quaternion retQuat = new Quaternion(quat[0], quat[1], quat[2], quat[3]);
        return retQuat;
    }

    /// <summary>
    /// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
    /// </summary>
    /// <param name="gx">
    /// Gyroscope x axis measurement in radians/s.
    /// </param>
    /// <param name="gy">
    /// Gyroscope y axis measurement in radians/s.
    /// </param>
    /// <param name="gz">
    /// Gyroscope z axis measurement in radians/s.
    /// </param>
    /// <param name="ax">
    /// Accelerometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="ay">
    /// Accelerometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="az">
    /// Accelerometer z axis measurement in any calibrated units.
    /// </param>
    /// <param name="mx">
    /// Magnetometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="my">
    /// Magnetometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="mz">
    /// Magnetometer z axis measurement in any calibrated units.
    /// </param>
    /// <remarks>
    /// Optimised for minimal arithmetic.
    /// Total ±: 160
    /// Total *: 172
    /// Total /: 5
    /// Total sqrt: 5
    /// </remarks> 
    public void calibrate(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        //float h = 0.0017f;
        //float g = 0.0035f;
        if(beginTime > 0.01f){
            float place = dt;
            dt = dt-beginTime;
            beginTime = place;
        } 
        //float dt = 0.1048f;
        //gx -= 25.048
        //gy += 60.47
        //gz -= 2.4
        //gx-=2.096f;
        //gy+=59.48f;
        //gz-=25.84f;
        calibrateValues[0]+=gx;
        calibrateValues[1]+=gy;
        calibrateValues[2]+=gz;
        calibrateValues[6]+=dt;
        //Debug.Log(dt);

        //print(""+gx+" "+gy+" "+gz);
        //print(beginTime);
        /*
        float predictionx = calibrateValues[0]+dt*calibrateValues[3];
        float predictiony = calibrateValues[1]+dt*calibrateValues[4];
        float predictionz = calibrateValues[2]+dt*calibrateValues[5];

        float residualx = gx - calibrateValues[3];
        float residualy = gy - calibrateValues[4];
        float residualz = gz - calibrateValues[5];

        calibrateValues[3] = h*(residualx*dt);
        calibrateValues[4] = h*(residualy*dt);
        calibrateValues[5] = h*(residualz*dt);
        float prevx=calibrateValues[0];
        float prevy=calibrateValues[1];
        float prevz=calibrateValues[2];
        calibrateValues[0] = calibrateValues[0]+g*residualx;
        calibrateValues[1] = calibrateValues[1]+g*residualy;
        calibrateValues[2] = calibrateValues[2]+g*residualz;*/
    }
    
    public void UpdateImu(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {   
        //float[][] init = new float[][]{new float[]{0.0f},new float[]{0.0f},new float[]{0.0f}};
        
        float h = 0.0017f;
        float g = 0.0030f;
        if(beginTime > 0.01f){
            float place = dt;
            dt = dt-beginTime;
            beginTime = place;
        } else {
            beginTime+=dt;
        }
        //float dt = 0.1048f;
        //gx -= 25.048
        //gy += 60.47
        //gz -= 2.4
        gx-=40.096f;
        gy+=55.48f;
        gz-=47.84f;
        print(""+gx+" "+gy+" "+gz);

        float predictionx = state[0]+dt*state[3];
        float predictiony = state[1]+dt*state[4];
        float predictionz = state[2]+dt*state[5];

        float residualx = gx - state[3];
        float residualy = gy - state[4];
        float residualz = gz - state[5];

        state[3] = h*(residualx*dt);
        state[4] = h*(residualy*dt);
        state[5] = h*(residualz*dt);
        float prevx=state[0];
        float prevy=state[1];
        float prevz=state[2];
        state[0] = state[0]+g*residualx;
        state[1] = state[1]+g*residualy;
        state[2] = state[2]+g*residualz;
        //Debug.Log(""+dt+""+state[0]+" "+state[1]+" "+state[2]+" "+state[3]+" "+state[4]+" "+state[5]);
        Quaternion rotation = Quaternion.Euler(state[2], state[0], state[1]);
        quat[0] = rotation.w;
        quat[1] = rotation.x;
        quat[2] = rotation.y;
        quat[3] = rotation.z;



        prevx=Mathf.Round(state[0]/1.8f-motorState[0]);
        prevy=Mathf.Round(state[1]/1.8f-motorState[1]);
        prevz=Mathf.Round(state[2]/1.8f-motorState[2]);
        motorState[0]+=(int)prevx;
        motorState[1]+=(int)prevy;
        motorState[2]+=(int)prevz;
        //print(prevx);
        int roll = (int)Mathf.Round(prevx);
        int pitch = (int)Mathf.Round(prevy);
        int yaw = (int)Mathf.Round(prevz);
        
        //byte roll = (byte)Mathf.Round((((Mathf.Round(prevx/1.8f))/200.0f)-(int)(Mathf.Round(prevx/1.8f)/200.0f))*200.0f);
        //byte pitch= (byte)Mathf.Round((((Mathf.Round(prevy/1.8f))/200.0f)-(int)(Mathf.Round(prevy/1.8f)/200.0f))*200.0f);
        //byte yaw= (byte)Mathf.Round((((Mathf.Round(prevz/1.8f))/200.0f)-(int)(Mathf.Round(prevz/1.8f)/200.0f))*200.0f);

        if((int)roll>100)roll = (300-roll);
        else if((int)roll>=0)roll = roll;
        else if((int)roll>=-100)roll=(Mathf.Abs(roll)+100);
        else if((int)roll>=-200)roll=(200-Mathf.Abs(roll));

        if((int)pitch>100)pitch =(300-pitch);
        else if((int)pitch>=0)pitch = pitch;
        else if((int)pitch>=-100)pitch=(Mathf.Abs(pitch)+100);
        else if((int)pitch>=-200)pitch=(200-Mathf.Abs(pitch));
        if((int)yaw>100)yaw = (300-yaw);
        else if((int)yaw>=0)yaw = yaw;
        else if((int)yaw>=-100)yaw=(Mathf.Abs(yaw)+100);
        else if((int)yaw>=-200)yaw=(200-Mathf.Abs(yaw));

        motorState[0] = (int)Mathf.Round(prevx);
        motorState[1] = (int)Mathf.Round(prevy);
        motorState[2] = (int)Mathf.Round(prevz);

        /*
        if(Mathf.Round(prevx/1.8f)>=0 && Mathf.Round(prevx/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             roll = (byte)Mathf.Round(prevx/1.8f);
        } else if(Mathf.Round(prevx/1.8f)<0 && Mathf.Round(prevx/1.8f)>100){
             roll = (byte)(Mathf.Round(prevx/1.8f)-200);
        } else if(Mathf.Round(prevx/1.8f)<-100){
             roll = (byte)Mathf.Round(prevx/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugX-Roll");
            
        }
        if(Mathf.Round(prevy/1.8f)>=0 && Mathf.Round(prevy/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             pitch = (byte)Mathf.Round(prevy/1.8f);
        } else if(Mathf.Round(prevx/1.8f)<0 && Mathf.Round(prevy/1.8f)>100){
             pitch = (byte)(Mathf.Round(prevy/1.8f)-200);
        } else if(Mathf.Round(prevy /1.8f)<-100){
             pitch = (byte)Mathf.Round(prevy/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugY-Pitch");
            
        }
        if(Mathf.Round(prevz/1.8f)>=0 && Mathf.Round(prevz/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             yaw = (byte)Mathf.Round(prevz/1.8f);
        } else if(Mathf.Round(prevz/1.8f)<0 && Mathf.Round(prevz/1.8f)>100){
             yaw = (byte)(Mathf.Round(prevz/1.8f)-200);
        } else if(Mathf.Round(prevz/1.8f)<-100){
             yaw = (byte)Mathf.Round(prevz/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugZ-Yaw");
            
        }*/

        byte[] array = {(byte)roll,(byte)pitch,(byte)yaw};
        //Debug.Log(roll+", "+pitch+", "+yaw);
        //Debug.Log(motorState[0]+", "+motorState[1]+", "+motorState[2]);
        
        //WriteToArduino(array,0,3);
        char a = 'X';
        byte[] test = {(byte)a};
        WriteToArduino(test,0,1);
        //uncomment one line above to activate motor sending

        /*
        float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2f * q1;
        float _2q2 = 2f * q2;
        float _2q3 = 2f * q3;
        float _2q4 = 2f * q4;
        float _2q1q3 = 2f * q1 * q3;
        float _2q3q4 = 2f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = (float)Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = (float)Mathf.Sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2f * q1 * mx;
        _2q1my = 2f * q1 * my;
        _2q1mz = 2f * q1 * mz;
        _2q2mx = 2f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = (float)Mathf.Sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2f * _2bx;
        _4bz = 2f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = 1f / (float)Mathf.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * SamplePeriod;
        q2 += qDot2 * SamplePeriod;
        q3 += qDot3 * SamplePeriod;
        q4 += qDot4 * SamplePeriod;
        norm = 1f / (float)Mathf.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        quat[0] = q1 * norm;
        quat[1] = q2 * norm;
        quat[2] = q3 * norm;
        quat[3] = q4 * norm;
        */
    }
    
    
    
    public void WriteToArduino(byte[] message, int offset, int count) {
        ARM.Write(message,offset,count);
        ARM.BaseStream.Flush();
        
    }
    
    public void StringToArduino(System.String toMotor) {
        //ARM.Open();

        if (!ARM.IsOpen) { ARM.Open(); }

         ARM.WriteLine(toMotor);
        ARM.BaseStream.Flush();
        //ARM.BaseStream.Flush();
        
    }/*
    public void StringToArduino() {
        //ARM.Open();
        if (!ARM.IsOpen) { ARM.Open(); }
         ARM.WriteLine("no");
        ARM.BaseStream.Flush();
        //ARM.BaseStream.Flush();
        
    }*/
}
