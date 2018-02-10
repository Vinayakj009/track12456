#include <stdarg.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>

#include <sys/time.h>
#include <unistd.h>

#ifdef NETBEANS
#include "ServerProgram/wiringPi.h"
#else
#include <wiringPi.h>
#endif

#include<time.h>

#include <string.h>
#include <sys/time.h>

#include <unistd.h>
#define DEBUG_MODE
#define MAXCLIENTS                  40
#define WRITE_BLOCK_RETRY_COUNT     2

#define INPUTPIN1   0
#define INPUTPIN2   1
#define INPUTPIN3   2
#define INPUTPIN4   3


#define INPUTPIN_ENABLE1   21
#define INPUTPIN_ENABLE2   22
#define INPUTPIN_ENABLE3   23
#define INPUTPIN_ENABLE4   24

//#define va_end(ap)              (void) 0
//#define va_start(ap, A)         (void) ((ap) = (((char *) &(A)) + (_bnd (A,_AUPBND))))
#define SERVER_STATUS_CODE_INIT                 -1
#define SERVER_STATUS_CODE_INITIATING            0
#define SERVER_STATUS_CODE_RUNNING_SIM_GR        3
#define SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET  2
#define SERVER_STATUS_CODE_RUNNING_RUN_GR        11
#define SERVER_STATUS_CODE_RUNNING_RUN_GR_RESET  10
#define SERVER_STATUS_CODE_RUNNING_SIM_SG        5
#define SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET  4
#define SERVER_STATUS_CODE_RUNNING_RUN_SG        13
#define SERVER_STATUS_CODE_RUNNING_RUN_SG_RESET  12
#define SERVER_STATUS_CODE_RUNNING_SIM_MM        7
#define SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET  6
#define SERVER_STATUS_CODE_RUNNING_RUN_MM        15
#define SERVER_STATUS_CODE_RUNNING_RUN_MM_RESET  14
#define SERVER_STATUS_CODE_RESET_MODE            1

#define PREFERRED_WORKING_DIRECTORY              ""

#define GR_SIMULATION_FILE_PATH                 "goldrushvalues.txt"
#define GR_TRACK_FILE_PATH                      "GoldRushTrackCount.txt"
#define MM_SIMULATION_FILE_PATH                 "mitmvalues.txt"
#define MM_TRACK_FILE_PATH                      "MITMTrackCount.txt"
#define SG_SIMULATION_FILE_PATH                 "salimgharvalues.txt"
#define SG_TRACK_FILE_PATH                      "SalimGharTrackCount.txt"
#define GR_VELOCITY_GRAPH_PATH                  "GoldRushVelocity.txt"
#define SENSOR_CALIBRATION_LOG_PATH             "CalibrationLogValues.txt"
#define RAMP_SETTINGS_STORAGE_FILE              "RampSettingsStoragefile.txt"



#define SIMULATION_DECIMATION_TIME          4000
#define INTERRUPT_MODE

#define SERVER_STATUS_CODE_TYPE_RUN_RESET       8
#define SERVER_STATUS_CODE_TYPE_RUN_STARTED     9
#define SERVER_STATUS_CODE_TYPE_SIM_RESET       0
#define SERVER_STATUS_CODE_TYPE_SIM_STARTED     1


#define APP_COMMAND_STRING_SHUTDOWN_SERVER            "SHUTDOWN"
#define APP_COMMAND_STRING_STOP_SERVER                "EXIT"
#define APP_COMMAND_STRING_GET_STATUS                 "SETTING"
#define APP_COMMAND_STRING_GET_LOCATION               "LOCATION"
#define APP_COMMAND_STRING_GET_CONNECTED_CLIENTS      "CLIENTS"
#define APP_COMMAND_STRING_GET_SENSOR                  "SENSOR"
#define APP_COMMAND_STRING_RESET_EXPERIENCE           "RESET"
#define APP_COMMAND_STRING_START_EXPERIENCE           "START"
#define APP_COMMAND_STRING_DISABLE_MEASUREMENT        "DBM"
#define APP_COMMAND_STRING_CALIBRATE_READING          "CALIBRATE"
#define APP_COMMAND_STRING_LOAD_CALIBRATION           "LOAD"
#define APP_COMMAND_STRING_STORE_BATTERY              "BATTERY"
#define APP_COMMAND_STRING_STORE_TEMPERATURE          "TEMPERATURE"
#define APP_COMMAND_STRING_STORE_PROXIMITY            "PROXIMITY"
#define APP_COMMAND_STRING_STORE_STATUS               "STATUS"
#define APP_COMMAND_STRING_STORE_NAME                 "NAME"
#define APP_COMMAND_STRING_GET_LOG                    "GETLOG"
#define APP_COMMAND_STRING_GET_LOG_FILE_COUNT         "GETLOGCOUNT"
#define APP_COMMAND_STRING_LOAD_VELOCITY_GRAPH        "LOADVEL"
#define APP_COMMAND_STRING_TOGGLE_VELOCITY_CORRECTION "TOGGLEV"
#define APP_COMMAND_STRING_ENABLE_MEASUREMENT         "EBM"
#define APP_COMMAND_STRING_ENABLE_CONTROL_PHONE       "ECP"
#define APP_COMMAND_STRING_SET_DATE                   "DATE"
#define APP_COMMAND_STRING_GET_VLOG                   "GETVLOG"
#define APP_COMMAND_STRING_GET_VLOG_FILE_COUNT        "GETVLOGCOUNT"
#define APP_COMMAND_STRING_SET_SENSORS_USED           "SETSENSORSUSED"
#define APP_COMMAND_STRING_GET_SENSORS_USED           "GETSENSORSUSED"
#define APP_COMMAND_STRING_GET_RAMP_TIME              "GETRAMPTIME"
#define APP_COMMAND_STRING_SET_RAMP_TIME              "SETRAMPTIME"
#define APP_COMMAND_STRING_PAUSE_RAMP                 "PAUSERAMP"
#define APP_COMMAND_STRING_START_RAMP                 "STARTRAMP"
#define APP_COMMAND_STRING_SET_RAMP_END_DISTANCE      "SRED"
#define APP_COMMAND_STRING_GET_RAMP_END_DISTANCE      "GRED"

#define GOLDRUSH_SERVER_MODE_STOPPED            0
#define GOLDRUSH_SERVER_MODE_RAMP               1
#define GOLDRUSH_SERVER_MODE_PAUSED             2
#define GOLDRUSH_SERVER_MODE_MAIN               3

#define DEFAULT_RAMP_END_TIME           40000
#define DEFAULT_RAMP_END_DISTANCE       105482

/* the code sends 0 for normal operation.
 * -1 for ramp mode
 * -2 for pause mode
 * -3 for Stop mode
 */


struct ClientStatus {
    int IsConnected;
    pthread_t ThreadId;
    int ClientSocketFileDiscriptor, ClientLength;
    struct sockaddr_in ClientAddress;
    int ClientIP[4], ResponseAvailable, DenyInput;
    float Temperature, Battery_Percentage;
    int Proximity, Status, WriteBlocked;
    char *Response, ResponseBuffer[10000], Command[256], Name[255];
    long Previous_Check_Time;
};

int Server_Status = SERVER_STATUS_CODE_INIT;
int GoldRush_Mode=GOLDRUSH_SERVER_MODE_STOPPED;
long Ramp_Time=40000,Ramp_End_Time;
float Ramp_End_Distance=105482,Ramp_Ignore_Distance;
/*@ToDo: have to setup a sensor setup check*/


void ProcessCommand(int ClientCount);

int StateIsType(int type) {
    return ((Server_Status & 0x09) == (type & 0x09));
}

int ThisStateIsType(int ThisStat, int Type) {
    return ((ThisStat & 0x09) == (Type & 0x09));
}

int Sensors_Used=0x0F;
float Sensors_Used_Count=4;

int MITMSimValues[150000], MITMNOV, MITMTrackCount = 2335;
float MITMIncrement = 0, MITMTrackLength = 100000.0;

float GoldRushIncrement = 0, GoldRushTrackLength = 840000.0;
int GoldRushSimValues[70000], GoldRushNOV, GoldRushTrackCount = 4000;

int SalimGharSimValues[150000], SalimGharNOV, SalimgharTrackCount = 8750;
float SalimGharIncrement = 0, SalimGharTrackLength = 267000.0;

int InputPinLog[4][150000], InputPinLogPointer[4], InputPinLogPointerPrevious[4];
char LogFilePath[30], pwd[30],VelocityLogFilePath[30];

float Velocity_Graph[6][200],GoldRushVelocityIncrement;
int Velocity_Graph_Calibration_Storage[200];
float Velocity_Error_Feedback_Weight=0.1,Velocity_Correction_Limit_Upper,Velocity_Correction_Limit_Lower;
int Velocity_Comparison[200];
int Velocity;
int Velocity_Storage[25],Velocity_Storage_Pointer;
int Velocity_Graph_Size,Velocity_Graph_Pointer;
int Velocity_Correction_Mode=1;

float distance;
int TranssferCount, OldManVideoTime = 0;

struct sigaction InputSigAction, ServerSigAction;
int ServerSocketFileDiscriptor, PortNumber = 1234, ClientCount = 0, MaximumFileDiscriptorID, ReUseSocket = 1, ConnectedClients = 0;
int SimulationStartTime, RunStartTime = 0, PreviousTransmitTime, PreviousTime, PreviousLocation = 0, location,previous_location;
struct sockaddr_in ServerAddress;
struct timeval Timer_Variable;
struct itimerval ServerTimer, InputTimer;
struct ClientStatus Clients[MAXCLIENTS + 1], FileCheckStruct;
fd_set ReadFileDiscriptors, ExceptFileDiscriptors, WriteFileDiscriptors;
char InputDataBuffer[256], OutputDataBuffer[256], Address[255], ConnectedAddresses[10000];

int stopeverything = 0, servercount = 0, inputcount = 0, connectedVR = 0;

int InputPipe[2], InputThreadProcessId, ServerPipe[2];

void InputCounter1(void) {
    InputPinLogPointer[0]++;
    InputPinLog[0][InputPinLogPointer[0]] = micros() - RunStartTime;
}

void InputCounter2(void) {
    InputPinLogPointer[1]++;
    InputPinLog[1][InputPinLogPointer[1]] = micros() - RunStartTime;
}

void InputCounter3(void) {
    InputPinLogPointer[2]++;
    InputPinLog[2][InputPinLogPointer[2]] = micros() - RunStartTime;
}

void InputCounter4(void) {
    InputPinLogPointer[3]++;
    InputPinLog[3][InputPinLogPointer[3]] = micros() - RunStartTime;
}

void printspecial(int priority, char *format, ...) {
    FILE *Writer;
    va_list arg;
    va_start(arg, *format);
    Writer = fopen(LogFilePath, "a+");
    if (priority > -1)
        fprintf(Writer,"%ld ",millis());
        vfprintf(Writer, format, arg);
    va_end(arg);
    fclose(Writer);
}

int previous_tlocation=-1;
float previous_tVelocity=-1;
float previous_tDistance=-1;
float previous_GoldRushIncrement=-1;
float previous_GoldRushVelocityIncrement=-1;

void PrintVelocityReadings(int tlocation,float tVelocity, float tDistance){
    FILE *Writer;
    if((previous_tlocation!=tlocation) ||
            (previous_tVelocity != tVelocity) ||
            (previous_tDistance != tDistance) ||
            (previous_GoldRushIncrement != GoldRushIncrement) ||
            (previous_GoldRushVelocityIncrement != GoldRushVelocityIncrement)) {
        Writer = fopen(VelocityLogFilePath, "a+");
        fprintf(Writer, "%ld,%d,%f,%f,%f,%f,%d\n", millis(), tlocation, tVelocity, tDistance, GoldRushIncrement, GoldRushVelocityIncrement, Velocity_Graph_Pointer);
        fclose(Writer);
        previous_tlocation = tlocation;
        previous_tVelocity = tVelocity;
        previous_tDistance = tDistance;
        previous_GoldRushIncrement = GoldRushIncrement;
        previous_GoldRushVelocityIncrement = GoldRushVelocityIncrement;
    }
}

void LogRunValues(char* heading, int logtype) {
    int iterator1, iterator2;
    FILE *Writer;
    Writer = fopen(LogFilePath, "a+");
    fprintf(Writer, "%s", heading);
    for (iterator1 = 0; iterator1 < logtype; iterator1++) {
        fprintf(Writer, "Sensor Log For Sensor %d", iterator1 + 1);
        for (iterator2 = 0; iterator2 < InputPinLogPointer[iterator1]; iterator2++) {
            fprintf(Writer, ",%d", InputPinLog[iterator1][iterator2]);
        }
        fprintf(Writer, "\n");
    }
    fclose(Writer);
}


void LogCalibrationValues(char* heading, int logtype) {
    int iterator1, iterator2;
    FILE *Writer;
    Writer = fopen(SENSOR_CALIBRATION_LOG_PATH, "a+");
    fprintf(Writer, "%s", heading);
    for (iterator1 = 0; iterator1 < logtype; iterator1++) {
        fprintf(Writer, "Calibration Log For Sensor %d", iterator1 + 1);
        for (iterator2 = 0; iterator2 < InputPinLogPointer[iterator1]; iterator2++) {
            fprintf(Writer, ",%d", InputPinLog[iterator1][iterator2]);
        }
        fprintf(Writer, "\n");
    }
    fclose(Writer);
}


void InitVelocityLogFile(void) {
    FILE *Reader, *Writer;
    int lognumber = 0;
    if (stat("vlog.txt", &FileCheckStruct) == 0) {
        Reader = fopen("vlog.txt", "r");
        fscanf(Reader, "%d\n", &lognumber);
        fclose(Reader);
    } else {
        system("mkdir vlog");
        lognumber = 0;
    }
    printspecial(1, "Starting Velocity Log File %d\n", lognumber++);
    if(lognumber>1500){
        lognumber=0;
    }
    sprintf(VelocityLogFilePath, "vlog/vlog%d.txt", lognumber);
    Writer = fopen("vlog.txt", "w");
    fprintf(Writer, "%d\n", lognumber);
    fclose(Writer);
}

void InitVelocityGraph(void){
    FILE *Reader;
    int i;
    float read_velocity,read_distance;
    int direction_pointer;
    printspecial(1, "Reading stored VelocityGraph\n");
    if(stat(GR_VELOCITY_GRAPH_PATH,&FileCheckStruct) == 0){
        Reader = fopen(GR_VELOCITY_GRAPH_PATH,"r");
        fscanf(Reader,"%d\n",&Velocity_Correction_Mode);
        fscanf(Reader,"%d\n",&Velocity_Graph_Size);
        fscanf(Reader,"%f\n",&Velocity_Error_Feedback_Weight);
        fscanf(Reader,"%f\n",&Velocity_Correction_Limit_Upper);
        fscanf(Reader,"%f\n",&Velocity_Correction_Limit_Lower);
        for(i=0;(i<Velocity_Graph_Size)&&(i<200);i++){
            fscanf(Reader,"%f,%f,%d\n",&read_distance,&read_velocity,&direction_pointer);
            Velocity_Graph[0][i]=read_distance;
            Velocity_Graph[1][i]=read_velocity;
            Velocity_Comparison[i]=direction_pointer;
            Velocity_Graph_Calibration_Storage[i]=0;
        }
        fscanf(Reader,"%d",&Sensors_Used);
    }else{
        Velocity_Graph_Size=0;
    }
    Velocity_Graph_Pointer = 0;
    Velocity_Storage_Pointer = 0;
    for (i = 0; i < 25; i++) {
        Velocity_Storage[i] = 0;
    }
    
    InitVelocityLogFile();
    GoldRushVelocityIncrement=GoldRushIncrement;
}

void StoreVelocityGraph(){
    
    FILE *Writer;
    int i;
    Writer= fopen(GR_VELOCITY_GRAPH_PATH,"w");
    fprintf(Writer,"%d\n%d\n%f\n%f\n%f\n",Velocity_Correction_Mode,Velocity_Graph_Size,Velocity_Error_Feedback_Weight,Velocity_Correction_Limit_Upper,Velocity_Correction_Limit_Lower);
    for(i=0;i<Velocity_Graph_Size;i++){
        fprintf(Writer,"%f,%f,%d\n",Velocity_Graph[0][i],Velocity_Graph[1][i],Velocity_Comparison[i]);
    }
    fprintf(Writer,"%d\n%f",Sensors_Used,Sensors_Used_Count);
    fclose(Writer);
}

void InitLogFile(void) {
    FILE *Reader, *Writer;
    int lognumber = 0;
    if (stat("log.txt", &FileCheckStruct) == 0) {
        Reader = fopen("log.txt", "r");
        fscanf(Reader, "%d\n", &lognumber);
        fclose(Reader);
    } else {
        system("mkdir log");
        lognumber = 0;
    }
    sprintf(LogFilePath, "log/log%d.txt", lognumber);
    if(lognumber>500){
        lognumber=0;
    }
    printspecial(1, "Starting Log File %d\n", lognumber++);
    Writer = fopen("log.txt", "w");
    fprintf(Writer, "%d\n", lognumber);
    fclose(Writer);
}


void sigquit() {
    printspecial(0, "My DADDY has Killed me!!!\n");
    stopeverything = 1;
}

void StoreCalibrationValuesToLog(){
    uint8_t data[1000];
    int i;
    sprintf(data,"cat %s >> %s",SENSOR_CALIBRATION_LOG_PATH,LogFilePath);
    system(data);
    sprintf(data,"echo \"Velocity Graph Size %d\" >> %s",Velocity_Graph_Size,LogFilePath);
    system(data);
    for(i=0;i<Velocity_Graph_Size;i++){
        sprintf(data,"echo \"Velocity Graph Point %d %f %f\" >> %s",i,Velocity_Graph[0][i],Velocity_Graph[1][i],LogFilePath);
        system(data);
    }
}

void LogCalibration() {
    int sending = 0;
    int LogCalibration_checker=200;
    uint8_t data[1000];
    sprintf(data,"rm %s",SENSOR_CALIBRATION_LOG_PATH);
    system(data);
    write(InputPipe[1], &LogCalibration_checker, sizeof (LogCalibration_checker));
    printspecial(0, "Logging Calibration\n");
    while (LogCalibration_checker != -10) {

        Timer_Variable.tv_sec = 0;
        Timer_Variable.tv_usec = 100;
        FD_ZERO(&ReadFileDiscriptors);
        FD_SET(ServerPipe[0], &ReadFileDiscriptors);
        MaximumFileDiscriptorID = ServerPipe[0];
        if (select(MaximumFileDiscriptorID + 1, &ReadFileDiscriptors, NULL, NULL, &Timer_Variable) == 0) {
            //            printspecial(0, "Waiting on sub thread%d\n", sending++);
            continue;
        }
        read(ServerPipe[0], &LogCalibration_checker, sizeof (LogCalibration_checker));
    }
    StoreCalibrationValuesToLog();
    //location = 0;
}

void ResetInput() {
    int sending = 0;
    printspecial(0, "Resetting input\n");
    write(InputPipe[1], &Server_Status, sizeof (Server_Status));
    InputPinLogPointer[0] = 0;
    InputPinLogPointer[1] = 0;
    InputPinLogPointer[2] = 0;
    InputPinLogPointer[3] = 0;
    location = 0;
    GoldRush_Mode=GOLDRUSH_SERVER_MODE_STOPPED;
//    location_difference=0;
    previous_location=0;
    while (location != -10) {

        Timer_Variable.tv_sec = 0;
        Timer_Variable.tv_usec = 100;
        FD_ZERO(&ReadFileDiscriptors);
        FD_SET(ServerPipe[0], &ReadFileDiscriptors);
        MaximumFileDiscriptorID = ServerPipe[0];
        if (select(MaximumFileDiscriptorID + 1, &ReadFileDiscriptors, NULL, NULL, &Timer_Variable) == 0) {
            //            printspecial(0, "Waiting on sub thread%d\n", sending++);
            continue;
        }
        read(ServerPipe[0], &location, sizeof (location));
    }
    location = 0;
}

void LoadTrackCount() {
    FILE *Reader;
    printspecial(11, "Loading track count\n");
    if (stat(SG_TRACK_FILE_PATH, &FileCheckStruct) == 0) {
        Reader = fopen(SG_TRACK_FILE_PATH, "r");
        fscanf(Reader, "%d\n", &SalimgharTrackCount);
        fscanf(Reader, "%f\n", &SalimGharTrackLength);
        fclose(Reader);
        if (SalimgharTrackCount < 1) {
            SalimgharTrackCount = 8750;
        }
    }

    SalimGharIncrement = SalimGharTrackLength / (float) SalimgharTrackCount;

    if (stat(GR_TRACK_FILE_PATH, &FileCheckStruct) == 0) {
        Reader = fopen(GR_TRACK_FILE_PATH, "r");
        fscanf(Reader, "%d\n", &GoldRushTrackCount);
        fscanf(Reader, "%f\n", &GoldRushTrackLength);
        fclose(Reader);
        if (GoldRushTrackCount < 1) {
            GoldRushTrackCount = 4000;
        }
    }
    GoldRushIncrement = GoldRushTrackLength / (float) GoldRushTrackCount;


    if (stat(MM_TRACK_FILE_PATH, &FileCheckStruct) == 0) {
        Reader = fopen(MM_TRACK_FILE_PATH, "r");
        fscanf(Reader, "%d\n", &MITMTrackCount);
        fscanf(Reader, "%f\n", &MITMTrackLength);
        fclose(Reader);
        if (MITMTrackCount < 1) {
            MITMTrackCount = 2335;
        }
    }
    MITMIncrement = MITMTrackLength / (float) MITMTrackCount;
}

void StoreTrackCount() {
    FILE *Writer;
    printspecial(11, "storing track count\n");
    Writer = fopen(SG_TRACK_FILE_PATH, "w");
    fprintf(Writer, "%d\n", SalimgharTrackCount);
    fprintf(Writer, "%f\n", SalimGharTrackLength);
    fclose(Writer);
    if (SalimgharTrackCount < 1) {
        SalimgharTrackCount = 8750;
    }

    Writer = fopen(GR_TRACK_FILE_PATH, "w");
    fprintf(Writer, "%d\n", GoldRushTrackCount);
    fprintf(Writer, "%f\n", GoldRushTrackLength);
    fclose(Writer);
    if (GoldRushTrackCount < 1) {
        GoldRushTrackCount = 4000;
    }


    Writer = fopen(MM_TRACK_FILE_PATH, "w");
    fprintf(Writer, "%d\n", MITMTrackCount);
    fprintf(Writer, "%f\n", MITMTrackLength);
    fclose(Writer);
    if (MITMTrackCount < 1) {
        MITMTrackCount = 4000;
    }
    LoadTrackCount();
}

int ReadSimValues(char *path, int *SimValues) {
    int length;
    long iterator1;
    FILE *Reader;
    if (stat(path, &FileCheckStruct) == 0) {
        printspecial(0, "Reading Sim Values from %s\n", path);
        Reader = fopen(path, "r");
        fscanf(Reader, "%d\n", &length);
        for (iterator1 = 0; iterator1 < length; iterator1++) {
            fscanf(Reader, "%d\n", (SimValues + iterator1));
        }
        fclose(Reader);
        printspecial(0, "The length was found to be %d\n", length);
        return length;
    } else {
        printspecial(0, "cound not find sim values %s\n", path);
    }
    return 0;
}

void StoreRampEndSettings(){
    char *path=RAMP_SETTINGS_STORAGE_FILE;
    FILE *Writer;
    Writer = fopen(path, "w");
    if ((Ramp_End_Distance < 0) || (Ramp_Time < 0)) {
        printspecial(0,"Errornous Ramp settings Ramd End Distance=%ld Ramp End Time =%ld, resetting to defaults",Ramp_End_Distance,Ramp_Time);
        Ramp_Time = DEFAULT_RAMP_END_TIME;
        Ramp_End_Distance = DEFAULT_RAMP_END_DISTANCE;
    }
    printspecial(0,"Storing Ramp settings Ramp End Distance=%f Ramp End Time =%ld\n",Ramp_End_Distance,Ramp_Time);
    fprintf(Writer, "Ramp End Time = %ld\nRamp End Distance = %f\n", Ramp_Time, Ramp_End_Distance);
    fclose(Writer);
    
}

void ReadRampSettings(){
    char *path=RAMP_SETTINGS_STORAGE_FILE;
    FILE *Reader;
    if(stat(path,&FileCheckStruct) == 0){
        printspecial(0, "Reading Ramp settings from %s\n",path);
        Reader=fopen(path,"r");
        fscanf(Reader,"Ramp End Time = %ld\nRamp End Distance = %f\n",&Ramp_Time,&Ramp_End_Distance);
        fclose(Reader);
    }else{
        
        printspecial(0, "Could not find ramp settings file %s, using defaults\n",path);
        Ramp_Time=DEFAULT_RAMP_END_TIME;
        Ramp_End_Distance=DEFAULT_RAMP_END_DISTANCE;
    }
    StoreRampEndSettings();
}

void Enable_Input_Lines(int new_sensor_settings){
    int i;
    Sensors_Used=new_sensor_settings;
    Sensors_Used_Count=0;
    for(i=1;i<0x10;i+=i){
        if(Sensors_Used&i){
            Sensors_Used_Count++;
        }
    }
//    cutoff_end_pulse=600*Sensors_Used_Count/4;
//    cutoff_velocity=44*Sensors_Used_Count/4;
    if(Sensors_Used&0x01){
        digitalWrite(INPUTPIN_ENABLE1,LOW);
    }else{
        digitalWrite(INPUTPIN_ENABLE1,HIGH);
    }
    if(Sensors_Used&0x02){
        digitalWrite(INPUTPIN_ENABLE2,LOW);
    }else{
        digitalWrite(INPUTPIN_ENABLE2,HIGH);
    }
    if(Sensors_Used&0x04){
        digitalWrite(INPUTPIN_ENABLE3,LOW);
    }else{
        digitalWrite(INPUTPIN_ENABLE3,HIGH);
    }
    if(Sensors_Used&0x08){
        digitalWrite(INPUTPIN_ENABLE4,LOW);
    }else{
        digitalWrite(INPUTPIN_ENABLE4,HIGH);
    }
    StoreVelocityGraph();
}

int main(int argc, char *argv[]) {
    char InputString[600];
    int iterator;
    long Time_Delay_For_Select=millis();
//    delay(10000);
    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET;

    for (iterator = 1; iterator < argc; iterator++) {
        if (strcmp(argv[iterator], "-pwd") == 0) {
            chdir(argv[++iterator]);
        } else if (strcmp(argv[iterator], "-portnumber") == 0) {
            iterator++;
            sscanf(argv[iterator], "%d", &PortNumber);
        } else if (strcmp(argv[iterator], "-RUN") == 0) {
            iterator++;
            if (strcmp(argv[iterator], "GR") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_GR_RESET;
                distance = -3;
            } else if (strcmp(argv[iterator], "SG") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_SG_RESET;
                distance = -2;
            } else if (strcmp(argv[iterator], "MM") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_MM_RESET;
                distance = 0;
            }
        } else if (strcmp(argv[iterator], "-SIM") == 0) {
            iterator++;
            if (strcmp(argv[iterator], "GR") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET;
                distance = -3;
            } else if (strcmp(argv[iterator], "SG") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET;
                distance = -2;
            } else if (strcmp(argv[iterator], "MM") == 0) {
                Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET;
                distance = 0;
            }
        }
    }
    InitLogFile();
    printspecial(0, "reading goldrush values\n");
    GoldRushNOV = ReadSimValues(GR_SIMULATION_FILE_PATH, GoldRushSimValues);

    printspecial(0, "reading salimghar values\n");
    SalimGharNOV = ReadSimValues(SG_SIMULATION_FILE_PATH, SalimGharSimValues);

    printspecial(0, "reading MM values\n");
    MITMNOV = ReadSimValues(MM_SIMULATION_FILE_PATH, MITMSimValues);
    
    ReadRampSettings();

    LoadTrackCount();
    
    InitVelocityGraph();

    printspecial(0, "Clearing clients\n");
    for (ClientCount = 0; ClientCount < MAXCLIENTS + 1; ClientCount++) {
        Clients[ClientCount].IsConnected = 0;
        Clients[ClientCount].DenyInput = 0;
        Clients[ClientCount].Battery_Percentage = 0;
        Clients[ClientCount].Temperature = 0;
        Clients[ClientCount].Proximity = 0;
        Clients[ClientCount].Status = 0;
        sprintf(Clients[ClientCount].Command, "AT");
    }
    StoreCalibrationValuesToLog();
    printspecial(0, "Starting Simulation\n");

    printspecial(0, "forking\n");

    int data = 0;
    pipe(InputPipe);
    pipe(ServerPipe);
    InputThreadProcessId = fork();

    if (wiringPiSetup() == -1) {
        printspecial(0, "Wiring Pi did not load\n");
        return 1;
    }
    pinMode(INPUTPIN1, INPUT);
    pinMode(INPUTPIN2, INPUT);
    pinMode(INPUTPIN3, INPUT);
    pinMode(INPUTPIN4, INPUT);
    pinMode(INPUTPIN_ENABLE1, OUTPUT);
    pinMode(INPUTPIN_ENABLE2, OUTPUT);
    pinMode(INPUTPIN_ENABLE3, OUTPUT);
    pinMode(INPUTPIN_ENABLE4, OUTPUT);
    Enable_Input_Lines(Sensors_Used);
    if (InputThreadProcessId == 0) {
        int CurrentTime, sucks = 0;
        char Command[255];
        close(InputPipe[1]);
        close(ServerPipe[0]);

        sprintf(Command, "gpio mode %d down", INPUTPIN1);
        system(Command);
        sprintf(Command, "gpio mode %d down", INPUTPIN2);
        system(Command);
        sprintf(Command, "gpio mode %d down", INPUTPIN3);
        system(Command);
        sprintf(Command, "gpio mode %d down", INPUTPIN4);
        system(Command);
        InputPinLogPointer[0] = 0;
        InputPinLogPointer[1] = 0;
        InputPinLogPointer[2] = 0;
        InputPinLogPointer[3] = 0;
        if (wiringPiISR(INPUTPIN1, INT_EDGE_BOTH, &InputCounter1) != 0) {
            printspecial(0, "Wiring Pi isr did not load on pin 1\n");
            return 1;
        }
        if (wiringPiISR(INPUTPIN2, INT_EDGE_BOTH, &InputCounter2) != 0) {
            printspecial(0, "Wiring Pi isr did not load on pin 2\n");
            return 1;
        }
        if (wiringPiISR(INPUTPIN3, INT_EDGE_BOTH, &InputCounter3) != 0) {
            printspecial(0, "Wiring Pi isr did not load on pin 3\n");
            return 1;
        }
        if (wiringPiISR(INPUTPIN4, INT_EDGE_BOTH, &InputCounter4) != 0) {
            printspecial(0, "Wiring Pi isr did not load on pin 4\n");
            return 1;
        }
        printspecial(0, "forked into child\n");
        read(InputPipe[0], &data, sizeof (data));
        SimulationStartTime = micros();
        PreviousTime = SimulationStartTime - SIMULATION_DECIMATION_TIME;
        PreviousTransmitTime = SimulationStartTime - 50000;
        while (1) {
            CurrentTime = micros();
            if (StateIsType(SERVER_STATUS_CODE_TYPE_SIM_STARTED)) {
                if ((CurrentTime - PreviousTime) > SIMULATION_DECIMATION_TIME) {
                    location = (CurrentTime - SimulationStartTime) / SIMULATION_DECIMATION_TIME; //&&(( CurrentTime - SimulationStartTime )%SIMULATION_DECIMATION_TIME)<1000){
                    if ((location - PreviousLocation) > 1) {
                        sucks += (location - PreviousLocation);
                        printspecial(1, "Simulation lagged behind %d,timediff =%d\n", sucks, CurrentTime - PreviousTransmitTime);
                        PreviousTransmitTime = CurrentTime;
                    }
                    PreviousTime = (SIMULATION_DECIMATION_TIME * location) + SimulationStartTime;
                    PreviousLocation = location;
                    write(ServerPipe[1], &location, sizeof (location));
                }
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_GR) {
                location = (InputPinLogPointer[0] +
                        InputPinLogPointer[1] +
                        InputPinLogPointer[2] +
                        InputPinLogPointer[3]);
                if (location != PreviousLocation) {
                    write(ServerPipe[1], &location, sizeof (location));
                    //                    printspecial(3, "location changed to %d\n", location);
                }
                PreviousLocation = location;
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_MM) {
                location = InputPinLogPointer[0];
                if (location != PreviousLocation) {
                    //                    printspecial(3, "location changed to %d\n", location);
                    write(ServerPipe[1], &location, sizeof (location));
                }
                PreviousLocation = location;
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_SG) {
                location = InputPinLogPointer[0];
                if (location != PreviousLocation) {
                    //                    printspecial(3, "location changed to %d\n", location);
                    write(ServerPipe[1], &location, sizeof (location));
                }
                PreviousLocation = location;
            }
            Timer_Variable.tv_sec = 0;
            Timer_Variable.tv_usec = 10;
            FD_ZERO(&ReadFileDiscriptors);
            FD_SET(InputPipe[0], &ReadFileDiscriptors);
            if (select(InputPipe[0] + 1, &ReadFileDiscriptors, NULL, NULL, &Timer_Variable) != 0) {
                int prevserver = Server_Status;
                read(InputPipe[0], &Server_Status, sizeof (Server_Status));
                printspecial(3, "Got this is server status in input coder %d\n", Server_Status);
                if(Server_Status==200){
                    int test=-10;
                    LogCalibrationValues("Logging Sensor Calibration Values\n", 4);
                    Server_Status=prevserver;
                    write(ServerPipe[1], &test, sizeof (test));
                }
		else
		{
                    switch (prevserver) {
                        case SERVER_STATUS_CODE_RUNNING_RUN_GR:
                            LogRunValues("Gold Rush Run Values\n", 4);
                            break;
                        case SERVER_STATUS_CODE_RUNNING_RUN_SG:
                            LogRunValues("Salimghar Run Values\n", 1);
                            break;
                        case SERVER_STATUS_CODE_RUNNING_RUN_MM:
                            LogRunValues("MITM Run Values\n", 4);
                            break;
                        default:
                            break;
                    }
                    //                StoreTrackCount();
                    if ((StateIsType(SERVER_STATUS_CODE_TYPE_RUN_STARTED))&&(ThisStateIsType(prevserver, SERVER_STATUS_CODE_TYPE_RUN_RESET))) {
                        InputPinLogPointer[0] = 0;
                        InputPinLogPointer[1] = 0;
                        InputPinLogPointer[2] = 0;
                        InputPinLogPointer[3] = 0;
                        RunStartTime = micros();
                        /* Attention remember top write code to store the logs*/
                    }
                    location = -10;
                    write(ServerPipe[1], &location, sizeof (location));
                    location = 0;
                    sleep(2);
                    SimulationStartTime = micros();
                    PreviousTime = SimulationStartTime;
                    PreviousLocation = 0;
                }

                usleep(1000);
            }
        }
        return;
    }
    close(InputPipe[0]);
    close(ServerPipe[1]);
    printspecial(0, "PID that i got was %d\n", InputThreadProcessId);
    system(InputDataBuffer);
    printspecial(0, "Creating socket\n");
    ServerSocketFileDiscriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (ServerSocketFileDiscriptor < 0) {
        printspecial(0, "Error opening Socket\n");
        return 1;
    }

    printspecial(0, "Using port number %d\n", PortNumber);
    bzero((char *) &ServerAddress, sizeof (ServerAddress));
    ServerAddress.sin_family = AF_INET;
    ServerAddress.sin_port = htons(PortNumber);
    ServerAddress.sin_addr.s_addr = INADDR_ANY;

    if (setsockopt(ServerSocketFileDiscriptor, SOL_SOCKET, SO_REUSEADDR, &ReUseSocket, sizeof (ReUseSocket)) == 1) {
        printspecial(0, "setsockopt");
        return 1;
    }
    if (bind(ServerSocketFileDiscriptor, (struct sockaddr *) &ServerAddress, sizeof (ServerAddress)) < 0) {
        printspecial(0, "Error on binding\n");
        return 1;
    }
    listen(ServerSocketFileDiscriptor, 5);
    int totaltransmits = 0, inputid = -2;
    write(InputPipe[1], &data, sizeof (data));
    Time_Delay_For_Select=millis();
    while (1) {
        data = 0;
        inputid = -2;
	long CTime;
        while (1) {
            Timer_Variable.tv_sec = 0;
            Timer_Variable.tv_usec = 10;
            FD_ZERO(&ReadFileDiscriptors);
            FD_SET(ServerPipe[0], &ReadFileDiscriptors);
            MaximumFileDiscriptorID = ServerPipe[0];
            if (select(MaximumFileDiscriptorID + 1, &ReadFileDiscriptors, NULL, NULL, &Timer_Variable) == 0) {
                break;
            }
            data++;
            read(ServerPipe[0], &location, sizeof (location));
            PreviousTime = micros();
            switch (Server_Status) {
                case SERVER_STATUS_CODE_RUNNING_SIM_SG:
                    if (location < SalimGharNOV) {
                        distance += SalimGharSimValues[location];
                    } else {
                        printspecial(1, "End of sim distance=%f, time=%d,total transmits=%d\n", distance, PreviousTime, totaltransmits);
                        Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET;
                        ResetInput();
                        distance = -2;
                    }
                    break;
                case SERVER_STATUS_CODE_RUNNING_SIM_GR:
                    if (location < GoldRushNOV) {
                        distance += GoldRushSimValues[location];
                    } else {
                        printspecial(1, "End of sim distance=%f, time=%d,total transmits=%d\n", distance, PreviousTime, totaltransmits);
                        Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET;
                        ResetInput();
                        distance = 0;
                    }
                    break;
                case SERVER_STATUS_CODE_RUNNING_SIM_MM:
                    if (location < MITMNOV) {
                        distance += MITMSimValues[location];
                    } else {
                        printspecial(1, "End of sim distance=%f, time=%d,total transmits=%d\n", distance, PreviousTime, totaltransmits);
                        Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET;
                        ResetInput();
                        distance = 0;
                    }
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_SG:
                    if (micros() < OldManVideoTime) {
                        distance = -1;
                    } else {
                        distance = SalimGharIncrement*location;

                    }
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_GR:
                    if(Velocity_Correction_Mode==1){
                        distance += GoldRushVelocityIncrement * (location-previous_location);
                    }else{
                        distance += GoldRushIncrement * (location-previous_location);
                    }
                    previous_location=location;
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_MM:
                    distance = MITMIncrement*location;
                    break;
                default:
                    if (location < 0) {
                        location = 0;
                    }
                    break;
            }
        }
        if ((distance == -1)&&(micros() > OldManVideoTime)&&((Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_SG) || (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_SG))) {
            distance = 0;
        }
        if(Server_Status==SERVER_STATUS_CODE_RUNNING_RUN_GR){
        switch(GoldRush_Mode){
            case GOLDRUSH_SERVER_MODE_STOPPED:
                if(location<1){
                    distance=-3;
                break;
                }
                Ramp_End_Time=millis()+Ramp_Time;
                GoldRush_Mode=GOLDRUSH_SERVER_MODE_RAMP;
            case GOLDRUSH_SERVER_MODE_PAUSED:
                distance=-2;
                break;
            case GOLDRUSH_SERVER_MODE_RAMP:
                if(Ramp_End_Time>millis())
                {
                    distance=-1;
                    break;
                }
                GoldRush_Mode=GOLDRUSH_SERVER_MODE_MAIN;
//                Ramp_Ignore_Distance=distance;
                distance=Ramp_End_Distance;
            case GOLDRUSH_SERVER_MODE_MAIN:
//                distance=distance+Ramp_End_Distance-Ramp_Ignore_Distance;
                break;
                  
        }
        }
        connectedVR = 0;
        totaltransmits++;
        Timer_Variable.tv_sec = 0;
        Timer_Variable.tv_usec = 1000;
        FD_ZERO(&ReadFileDiscriptors);
        FD_ZERO(&ExceptFileDiscriptors);
        FD_ZERO(&WriteFileDiscriptors);
        FD_SET(ServerSocketFileDiscriptor, &ReadFileDiscriptors);
        FD_SET(ServerSocketFileDiscriptor, &ExceptFileDiscriptors);
        MaximumFileDiscriptorID = ServerSocketFileDiscriptor;
        if(Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_GR)
            if ((Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_GR)&&(Velocity_Correction_Mode == 1)) {
                    Velocity = location - Velocity_Storage[Velocity_Storage_Pointer];
                    Velocity_Storage[Velocity_Storage_Pointer] = location;
                    Velocity_Storage_Pointer = (Velocity_Storage_Pointer + 1) % 25;
                if ((Velocity_Graph_Pointer < 1) || (GoldRush_Mode == GOLDRUSH_SERVER_MODE_MAIN)) {
                    if (Velocity_Graph_Pointer < Velocity_Graph_Size - 1) {
                        float Correction_Velocity = Velocity_Graph[1][Velocity_Graph_Pointer];
                        Correction_Velocity = Correction_Velocity * ((float) Sensors_Used_Count) / 4.0;
                        if (((Velocity > Correction_Velocity)
                                &&(Velocity_Comparison[Velocity_Graph_Pointer] == 1))
                                || ((Velocity < Correction_Velocity)
                                &&(Velocity_Comparison[Velocity_Graph_Pointer] == 0))) {
                            float next_location_count = -1, error, ratio;
                            error = distance - (Velocity_Graph[0][Velocity_Graph_Pointer]);
                            error = Velocity_Error_Feedback_Weight * error;
                            next_location_count = ((Velocity_Graph[0][Velocity_Graph_Pointer + 1] + error - Velocity_Graph[0][Velocity_Graph_Pointer]) / GoldRushIncrement);
                            GoldRushVelocityIncrement = (Velocity_Graph[0][Velocity_Graph_Pointer + 1] - Velocity_Graph[0][Velocity_Graph_Pointer]) / next_location_count;
                            ratio = GoldRushIncrement / GoldRushIncrement;
                            if (ratio > Velocity_Correction_Limit_Upper) {
                                GoldRushVelocityIncrement = Velocity_Correction_Limit_Upper*GoldRushIncrement;
                            } else if (ratio < Velocity_Correction_Limit_Lower) {
                                GoldRushVelocityIncrement = Velocity_Correction_Limit_Lower*GoldRushIncrement;
                            } else {
                                Velocity_Graph_Calibration_Storage[Velocity_Graph_Pointer] = location;
                            }
                            Velocity_Graph_Pointer++;
                        }
                    }
                }
                PrintVelocityReadings(location, Velocity, distance);
            }

        bzero(OutputDataBuffer, 256);
        sprintf(OutputDataBuffer, "%d\n", (int) distance);
        sprintf(ConnectedAddresses, "");
        for (ClientCount = 0; ClientCount < MAXCLIENTS; ClientCount++) {
            if (Clients[ClientCount].IsConnected == 1) {
                FD_SET(Clients[ClientCount].ClientSocketFileDiscriptor, &ReadFileDiscriptors);
                FD_SET(Clients[ClientCount].ClientSocketFileDiscriptor, &WriteFileDiscriptors);
                FD_SET(Clients[ClientCount].ClientSocketFileDiscriptor, &ExceptFileDiscriptors);
                if (Clients[ClientCount].ClientSocketFileDiscriptor > MaximumFileDiscriptorID) {
                    MaximumFileDiscriptorID = Clients[ClientCount].ClientSocketFileDiscriptor;
                }
                if (Clients[ClientCount].DenyInput == 0) {
                    sprintf(ConnectedAddresses, "%sIP=%d:%d:%d:%d B=%.02f T=%.02f S=%d P=%d N=%s\n", ConnectedAddresses, Clients[ClientCount].ClientIP[0], Clients[ClientCount].ClientIP[1], Clients[ClientCount].ClientIP[2], Clients[ClientCount].ClientIP[3], Clients[ClientCount].Battery_Percentage, Clients[ClientCount].Temperature, Clients[ClientCount].Status, Clients[ClientCount].Proximity, Clients[ClientCount].Name);
                    connectedVR++;
                }
            }
        }
	//hallay
	CTime=Time_Delay_For_Select-millis();
        if(CTime>0)
        {
            delay(CTime);
        }
	CTime=millis();
        select(MaximumFileDiscriptorID + 1, &ReadFileDiscriptors, &WriteFileDiscriptors, &ExceptFileDiscriptors, &Timer_Variable);
        Time_Delay_For_Select+=40;
	if (FD_ISSET(0, &ReadFileDiscriptors)) {
            char datac[60];
            scanf("%s", &datac);
        } else {
            sprintf(InputString, "");
        }
        if (FD_ISSET(ServerSocketFileDiscriptor, &ReadFileDiscriptors)) {
            for (ClientCount = 0; ClientCount < MAXCLIENTS + 1; ClientCount++) {
                if (Clients[ClientCount].IsConnected == 0) {
                    break;
                } else {
                    //						printspecial(0,"Client point %d is busy %d\n",ClientCount,Clients[ClientCount].IsConnected);
                }
            }

            ConnectedClients++;
            Clients[ClientCount].ClientLength = sizeof (Clients[ClientCount].ClientAddress);
            Clients[ClientCount].ClientSocketFileDiscriptor = accept(ServerSocketFileDiscriptor, (struct sockaddr *) &(Clients[ClientCount].ClientAddress), &(Clients[ClientCount].ClientLength));

            sprintf(Address, "%s", inet_ntoa(Clients[ClientCount].ClientAddress.sin_addr));

            sscanf(Address,
                    "%d.%d.%d.%d",
                    &(Clients[ClientCount].ClientIP[0]),
                    &(Clients[ClientCount].ClientIP[1]),
                    &(Clients[ClientCount].ClientIP[2]),
                    &(Clients[ClientCount].ClientIP[3]));
            printspecial(0,
                    "Accepting Client on %d, with ip address %d.%d.%d.%d\n",
                    ClientCount,
                    Clients[ClientCount].ClientIP[0],
                    Clients[ClientCount].ClientIP[1],
                    Clients[ClientCount].ClientIP[2],
                    Clients[ClientCount].ClientIP[3]);
            sprintf(Clients[ClientCount].Name, "");
            sprintf(Clients[ClientCount].Command, "");
            sprintf(Clients[ClientCount].ResponseBuffer, "");
            Clients[ClientCount].Battery_Percentage = 0;
            Clients[ClientCount].Temperature = 0;
            Clients[ClientCount].Status = 0;
            Clients[ClientCount].Proximity = 0;
            Clients[ClientCount].ResponseAvailable = 0;
            Clients[ClientCount].DenyInput = 0;
            Clients[ClientCount].Previous_Check_Time=millis();

            if (Clients[ClientCount].ClientSocketFileDiscriptor < 0) {
                printspecial(0, "ERROR on accept\n");
            } else {
                if (ClientCount != MAXCLIENTS) {
                    Clients[ClientCount].IsConnected = 1;
                    Clients[ClientCount].WriteBlocked = 0;

                } else {
                    printspecial(0, "cannot Accomodate any more clients\n");
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                }
            }
        }
        for (ClientCount = 0; ClientCount < MAXCLIENTS; ClientCount++) {
            if ((Clients[ClientCount].IsConnected != 1))
                continue;
            if (FD_ISSET(Clients[ClientCount].ClientSocketFileDiscriptor, &ExceptFileDiscriptors)) {
                printspecial(0, "Exception Connection closed, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                close(Clients[ClientCount].ClientSocketFileDiscriptor);
                Clients[ClientCount].IsConnected = 0;
                ConnectedClients--;
                continue;
            }
            int errorsoc = 0;
                socklen_t len = sizeof (errorsoc);
                int retval = getsockopt(Clients[ClientCount].ClientSocketFileDiscriptor
                        , SOL_SOCKET
                        , SO_ERROR
                        , &errorsoc
                        , &len);
                if ((retval != 0) || (errorsoc != 0)) {
                    printspecial(0, "Read error Connection closed, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                    Clients[ClientCount].IsConnected = 0;
                    ConnectedClients--;
                    continue;
                }
            if (FD_ISSET(Clients[ClientCount].ClientSocketFileDiscriptor, &ReadFileDiscriptors)) {
                
                bzero(InputDataBuffer, 256);
                TranssferCount = read(Clients[ClientCount].ClientSocketFileDiscriptor, InputDataBuffer, 255);
                if (TranssferCount < 1) {
                    printspecial(0, "Read Number Connection closed, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                    Clients[ClientCount].IsConnected = 0;
                    ConnectedClients--;
                    continue;
                }
                sprintf(Clients[ClientCount].Command, "%s", InputDataBuffer);
                printspecial(0, "Message received from ip %d with client id %d : %s\n", Clients[ClientCount].ClientIP[3], ClientCount, InputDataBuffer);
                Clients[ClientCount].Previous_Check_Time=millis();

            }
            if (!FD_ISSET(Clients[ClientCount].ClientSocketFileDiscriptor, &WriteFileDiscriptors)) {
                if (Clients[ClientCount].WriteBlocked < WRITE_BLOCK_RETRY_COUNT) {
                    Clients[ClientCount].WriteBlocked++;
                    printspecial(0, "Write Blocked, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                    continue;
                } else {
                    printspecial(0, "Write Blocked Connection closed, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                    Clients[ClientCount].IsConnected = 0;
                    ConnectedClients--;
                    continue;
                }
            } else {
                Clients[ClientCount].WriteBlocked = 0;
            }
            int time1, time2;
            char *data_writer,OutData[500];
            bzero(OutData, 500);
            time1 = micros();
            data_writer=OutData;
            if (Clients[ClientCount].DenyInput == 0) {
                sprintf(OutData, "%s", OutputDataBuffer);
                Clients[ClientCount].Previous_Check_Time=CTime;
            }
            if((Clients[ClientCount].Previous_Check_Time+940)<CTime){
                Clients[ClientCount].Previous_Check_Time=CTime;
                if(Clients[ClientCount].DenyInput==2){
                    sprintf(OutData,"");
                    if(strcmp(Clients[ClientCount].Command,"AT")!=0){
                        sprintf(OutData,"%s",Clients[ClientCount].Command);
                    }
                    sprintf(Clients[ClientCount].Command,"%s\n%s\n%s\n%s\n%s",OutData,APP_COMMAND_STRING_GET_STATUS,
                            APP_COMMAND_STRING_GET_LOCATION,
                            APP_COMMAND_STRING_GET_CONNECTED_CLIENTS,
                            APP_COMMAND_STRING_GET_RAMP_TIME);
                    sprintf(OutData,"");
                }
            }
            if (Clients[ClientCount].ResponseAvailable) {
                sprintf(Clients[ClientCount].Response, "%s%s", Clients[ClientCount].Response,OutData);
                Clients[ClientCount].ResponseAvailable = 0;
                Clients[ClientCount].Previous_Check_Time=CTime;
                data_writer=Clients[ClientCount].Response;
            }
            TranssferCount = write(Clients[ClientCount].ClientSocketFileDiscriptor, data_writer, strlen(data_writer));
            time2 = micros();
            if ((TranssferCount < strlen(data_writer))) {
                printspecial(0, "Write Incomplete Connection closed, Client %d ip %d\n", ClientCount, Clients[ClientCount].ClientIP[3]);
                close(Clients[ClientCount].ClientSocketFileDiscriptor);
                Clients[ClientCount].IsConnected = 0;
                ConnectedClients--;
            }
        }
        for (ClientCount = 0; ClientCount < MAXCLIENTS; ClientCount++) {
            if (strcmp(Clients[ClientCount].Command, "AT") != 0) {
                ProcessCommand(ClientCount);
                sprintf(Clients[ClientCount].Command, "AT");
            }
        }
    }

}

void ProcessCommand(int ClientCount) {
    int i = 0;
    uint8_t Printed[60];
    bzero(Printed,60);
    char Command[256], Parameters[2][256];
    Clients[ClientCount].ResponseAvailable = 1;
    Clients[ClientCount].Response = Clients[ClientCount].ResponseBuffer;
    while (i < strlen(Clients[ClientCount].Command)) {
        sprintf(Clients[ClientCount].Response, "Please Clarify\n");
        sscanf(Clients[ClientCount].Command + i, "%s", Command);
        i += strlen(Command) + 1;
        if (strcmp(Command, APP_COMMAND_STRING_STOP_SERVER) == 0) {
            if(Printed[0]==1){
                continue;
            }
            Printed[0]=1;
            write(Clients[ClientCount].ClientSocketFileDiscriptor, "Stopping Server\n", strlen("Stopping Server\n"));
            for (ClientCount = 0; ClientCount < MAXCLIENTS; ClientCount++) {
                if (Clients[ClientCount].IsConnected == 1) {
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                }
            }
            close(ServerSocketFileDiscriptor);
            if (StateIsType(SERVER_STATUS_CODE_TYPE_RUN_STARTED)) {
                Server_Status &= 0x0E;
                ResetInput();
                delay(500);
            }
            kill(InputThreadProcessId, SIGQUIT);
            printspecial(0, "Shutting down the server\n");
            delay(10);
            exit(0);
        } else if (strcmp(Command, APP_COMMAND_STRING_SHUTDOWN_SERVER) == 0) {
            
            if(Printed[1]==1){
                continue;
            }
            Printed[1]=1;
            write(Clients[ClientCount].ClientSocketFileDiscriptor, "Shutting Down The Server\n", strlen("Shutting Down The Server\n"));
            for (ClientCount = 0; ClientCount < MAXCLIENTS; ClientCount++) {
                if (Clients[ClientCount].IsConnected == 1) {
                    close(Clients[ClientCount].ClientSocketFileDiscriptor);
                }
            }
            close(ServerSocketFileDiscriptor);
            if (StateIsType(SERVER_STATUS_CODE_TYPE_RUN_STARTED)) {
                Server_Status &= 0x0E;
                ResetInput();
                delay(500);
            }
            printspecial(0, "Shutting down the Pi\n");
            kill(InputThreadProcessId, SIGQUIT);
            delay(10);
            system("sudo shutdown -h now");
            exit(0);
        } else if (strcmp(Command, APP_COMMAND_STRING_RESET_EXPERIENCE) == 0) {
            
            if(Printed[2]==1){
                continue;
            }
            Printed[2]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                i += strlen(Parameters[0]) + 1;
                if (i <= strlen(Clients[ClientCount].Command)) {
                    sscanf(Clients[ClientCount].Command + i, "%s", Parameters[1]);
                    i += strlen(Parameters[1]) + 1;
                }
            }
            if (strcmp(Parameters[0], "SIM") == 0) {
                if (strcmp(Parameters[1], "GR") == 0) {
                    distance = 0;
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running Gold rush simulation\n");
                    ResetInput();
                } else if (strcmp(Parameters[1], "MM") == 0) {
                    distance = 0;
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running MITM simulation\n");
                    ResetInput();
                } else if (strcmp(Parameters[1], "SG") == 0) {
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running SalinGhar simulation\n");
                    distance = -2;
                    ResetInput();
                }
            } else if (strcmp(Parameters[0], "RUN") == 0) {
                if (strcmp(Parameters[1], "GR") == 0) {
                    distance = -3;
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_GR_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running Gold rush sensor\n");
                    ResetInput();
                } else if (strcmp(Parameters[1], "MM") == 0) {
                    distance = 0;
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_MM_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running MITM sensor\n");
                    ResetInput();
                } else if (strcmp(Parameters[1], "SG") == 0) {
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_SG_RESET;
                    sprintf(Clients[ClientCount].Response, "Resetting count and distance and running SalimGhar sensor\n");
                    distance = -2;
                    ResetInput();
                }
            }
        } else if (strcmp(Command, APP_COMMAND_STRING_START_EXPERIENCE) == 0) {
            
            if(Printed[3]==1){
                continue;
            }
            Printed[3]=1;
            switch (Server_Status) {
                case SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_GR;
                    distance = 0;
                    ResetInput();
                    sprintf(Clients[ClientCount].Response, "Started GoldRush simulation\n");
                    break;
                case SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_SG;
                    ResetInput();
                    OldManVideoTime = micros();
                    OldManVideoTime += 17000000;
                    distance = -1;
                    sprintf(Clients[ClientCount].Response, "Started Salimghar simulation\n");
                    break;
                case SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_SIM_MM;
                    distance = 0;
                    ResetInput();
                    sprintf(Clients[ClientCount].Response, "Started MITM simulation\n");
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_GR_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_GR;
                    InitVelocityGraph();
                    Velocity=0;
                    distance = -3;
                    ResetInput();
                    sprintf(Clients[ClientCount].Response, "Started GoldRush sensor\n");
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_SG_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_SG;
                    ResetInput();
                    OldManVideoTime = micros();
                    OldManVideoTime += 17000000;
                    distance = -1;
                    sprintf(Clients[ClientCount].Response, "Started Salimghar sensor\n");
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_MM_RESET:
                    Server_Status = SERVER_STATUS_CODE_RUNNING_RUN_MM;
                    distance = 0;
                    ResetInput();
                    sprintf(Clients[ClientCount].Response, "Started MITM sensor\n");
                    break;
            }
        } else if (strcmp(Command, APP_COMMAND_STRING_GET_STATUS) == 0) {
            
            if(Printed[4]==1){
                continue;
            }
            Printed[4]=1;
            sprintf(Clients[ClientCount].Response, "Mode ");
            Clients[ClientCount].Response += strlen(Clients[ClientCount].Response);
            if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_GR) {
                sprintf(Clients[ClientCount].Response, "SIM GR S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_GR_RESET) {
                sprintf(Clients[ClientCount].Response, "SIM GR R\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_GR) {
                sprintf(Clients[ClientCount].Response, "RUN GR S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_GR_RESET) {
                sprintf(Clients[ClientCount].Response, "RUN GR R\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_SG) {
                sprintf(Clients[ClientCount].Response, "SIM SG S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_SG_RESET) {
                sprintf(Clients[ClientCount].Response, "SIM SG R\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_SG) {
                sprintf(Clients[ClientCount].Response, "RUN SG S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_SG_RESET) {
                sprintf(Clients[ClientCount].Response, "RUN SG R\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_MM) {
                sprintf(Clients[ClientCount].Response, "SIM MM S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_SIM_MM_RESET) {
                sprintf(Clients[ClientCount].Response, "SIM MM R\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_MM) {
                sprintf(Clients[ClientCount].Response, "RUN MM S\n");
            } else if (Server_Status == SERVER_STATUS_CODE_RUNNING_RUN_MM_RESET) {
                sprintf(Clients[ClientCount].Response, "RUN MM R\n");
            } else {
                sprintf(Clients[ClientCount].Response, "FL\n");
            }
        } else if (strcmp(Command, APP_COMMAND_STRING_GET_LOCATION) == 0) {
            
            if(Printed[5]==1){
                continue;
            }
            Printed[5]=1;
            
            int data = 0;
            if (digitalRead(INPUTPIN1)) {
                data |= 1;
            }
            if (digitalRead(INPUTPIN2)) {
                data |= 2;
            }
            if (digitalRead(INPUTPIN3)) {
                data |= 4;
            }
            if (digitalRead(INPUTPIN4)) {
                data |= 8;
            }
            sprintf(Clients[ClientCount].Response, "Count=%d,distance=%f,vLocation=%d,Sensor=%d,Sensors_Used=%d\n", location, distance,Velocity_Graph_Pointer,data,Sensors_Used);
        } else if (strcmp(Command, APP_COMMAND_STRING_GET_SENSOR) == 0) {
            
            if(Printed[6]==1){
                continue;
            }
            Printed[6]=1;
            int data = 0;
            if (digitalRead(INPUTPIN1)) {
                data |= 1;
            }
            if (digitalRead(INPUTPIN2)) {
                data |= 2;
            }
            if (digitalRead(INPUTPIN3)) {
                data |= 4;
            }
            if (digitalRead(INPUTPIN4)) {
                data |= 8;
            }
            sprintf(Clients[ClientCount].Response, "Sensor=%d\n", data);
        } else if (strcmp(Command, APP_COMMAND_STRING_GET_CONNECTED_CLIENTS) == 0) {
            
            if(Printed[7]==1){
                continue;
            }
            Printed[7]=1;
            sprintf(Clients[ClientCount].Response, "Connected Client Count =%d\n%s", connectedVR, ConnectedAddresses);
            
        } else if (strcmp(Command, APP_COMMAND_STRING_CALIBRATE_READING) == 0) {
            
            if(Printed[8]==1){
                continue;
            }
            Printed[8]=1;
            switch (Server_Status) {
                case SERVER_STATUS_CODE_RUNNING_RUN_GR:
                    GoldRushTrackCount = location;
                    GoldRushIncrement = GoldRushTrackLength / (float) GoldRushTrackCount;
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_SG:
                    SalimgharTrackCount = location;
                    SalimGharIncrement = SalimGharTrackLength / (float) SalimgharTrackCount;
                    break;
                case SERVER_STATUS_CODE_RUNNING_RUN_MM:
                    MITMTrackCount = location;
                    MITMIncrement = MITMTrackLength / (float) MITMTrackCount;
                    break;
            }
            sprintf(Clients[ClientCount].Response, "Storing Track Count\n Salimghar %d,%f\nGoldRush %d,%f\n MITM %d,%f\n", SalimgharTrackCount, SalimGharTrackLength, GoldRushTrackCount, GoldRushTrackLength, MITMTrackCount, MITMTrackLength);
            if((Server_Status==SERVER_STATUS_CODE_RUNNING_RUN_GR)&&(Velocity_Correction_Mode==1)){
                Clients[ClientCount].Response += strlen(Clients[ClientCount].Response);
                if(Velocity_Graph_Pointer==Velocity_Graph_Size-1){
                    int i;
                    sprintf(Clients[ClientCount].Response, "Storing Velocity Graph %d\n",Velocity_Graph_Size);
                    for(i=1;i<Velocity_Graph_Pointer;i++){
                        Velocity_Graph[0][i]=Velocity_Graph_Calibration_Storage[i]*GoldRushIncrement;
                    }
                    StoreVelocityGraph();
                }else{
                sprintf(Clients[ClientCount].Response, "Could not generate velocity graph %d\n",Velocity_Graph_Pointer);
                }
            }
            StoreTrackCount();
            LogCalibration();
        } else if (strcmp(Command, APP_COMMAND_STRING_LOAD_CALIBRATION) == 0) {
            
            if(Printed[9]==1){
                continue;
            }
            Printed[9]=1;
            LoadTrackCount();
            sprintf(Clients[ClientCount].Response, "Loaded Track Count\n Salimghar %d,%f\nGoldRush %d,%f\n MITM %d,%f\n", SalimgharTrackCount, SalimGharTrackLength, GoldRushTrackCount, GoldRushTrackLength, MITMTrackCount, MITMTrackLength);
        } else if (strcmp(Command, APP_COMMAND_STRING_DISABLE_MEASUREMENT) == 0) {
            
            if(Printed[10]==1){
                continue;
            }
            Printed[10]=1;
            sprintf(Clients[ClientCount].Response, "Disabling Measurement\n");
            Clients[ClientCount].DenyInput = 1;
        } else if (strcmp(Command, APP_COMMAND_STRING_STORE_BATTERY) == 0) {
            
            if(Printed[11]==1){
                continue;
            }
            Printed[11]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%f", &(Clients[ClientCount].Battery_Percentage));
                i += strlen(Parameters[0]) + 1;
            }
            sprintf(Clients[ClientCount].Response, "");
        } else if (strcmp(Command, APP_COMMAND_STRING_STORE_TEMPERATURE) == 0) {
            if(Printed[12]==1){
                continue;
            }
            Printed[12]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%f", &(Clients[ClientCount].Temperature));
                i += strlen(Parameters[0]) + 1;
            }
            sprintf(Clients[ClientCount].Response, "");
        } else if (strcmp(Command, APP_COMMAND_STRING_STORE_PROXIMITY) == 0) {
            
            if(Printed[13]==1){
                continue;
            }
            Printed[13]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%d", &(Clients[ClientCount].Proximity));
                i += strlen(Parameters[0]) + 1;
            }
            sprintf(Clients[ClientCount].Response, "");
        } else if (strcmp(Command, APP_COMMAND_STRING_STORE_STATUS) == 0) {
            
            if(Printed[14]==1){
                continue;
            }
            Printed[14]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%d", &(Clients[ClientCount].Status));
                i += strlen(Parameters[0]) + 1;
            }
            sprintf(Clients[ClientCount].Response, "");
        } else if (strcmp(Command, APP_COMMAND_STRING_STORE_NAME) == 0) {
            
            if(Printed[15]==1){
                continue;
            }
            Printed[15]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%s", Clients[ClientCount].Name);
                i += strlen(Parameters[0]) + 1;
            }
            sprintf(Clients[ClientCount].Response, "");
        } else if (strcmp(Command, APP_COMMAND_STRING_GET_LOG) == 0) {
            
            if(Printed[16]==1){
                continue;
            }
            Printed[16]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                FILE *reader;
                int nol, iterator;
                char Path[20];
                reader = fopen("log.txt", "r");
                fscanf(reader, "%d\n", &nol);
                fclose(reader);
                nol--;
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                sscanf(Parameters[0], "%d", &iterator);
                i += strlen(Parameters[0]) + 1;
                if (iterator > nol) {
                    sprintf(Clients[ClientCount].Response, "Log only upto %d", nol);
                } else {
                    InputThreadProcessId = fork();
                    if (InputThreadProcessId == 0) {
                        sprintf(Path, "nc -l 1236<log/log%d.txt", iterator);
                        system(Path);
                        exit(0);
                    }
                    delay(20);
                    sprintf(Clients[ClientCount].Response, "Log server created\n");
                }
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_GET_LOG_FILE_COUNT)==0){
            int lognumber=0;
                
            if(Printed[17]==1){
                continue;
            }
            Printed[17]=1;
            if (stat("log.txt", &FileCheckStruct) == 0) {
                FILE *Reader;
                Reader = fopen("log.txt", "r");
                fscanf(Reader, "%d\n", &lognumber);
                fclose(Reader);    
            }
            sprintf(Clients[ClientCount].Response, "Log number %d\n",lognumber);
        }else if (strcmp(Command,APP_COMMAND_STRING_LOAD_VELOCITY_GRAPH)==0){
                
            if(Printed[18]==1){
                continue;
            }
            Printed[18]=1;
            if(Server_Status!=SERVER_STATUS_CODE_RUNNING_RUN_GR){
            InitVelocityGraph();
                sprintf(Clients[ClientCount].Response, "Velocity Graph Reloaded\n");
            }else{
                sprintf(Clients[ClientCount].Response, "Could not reload Velocity Graph\n");
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_TOGGLE_VELOCITY_CORRECTION)==0){
            
            if(Printed[19]==1){
                continue;
            }
            Printed[19]=1;
            if(Server_Status!=SERVER_STATUS_CODE_RUNNING_RUN_GR){
                Velocity_Correction_Mode=1-Velocity_Correction_Mode;
                sprintf(Clients[ClientCount].Response,"Velocity Correction Mode %s\n",(Velocity_Correction_Mode==1)?"Started":"Stopped");
            }else{
                 sprintf(Clients[ClientCount].Response,"Cannot toggle velocity mode\n");
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_ENABLE_MEASUREMENT)==0){
                
            if(Printed[20]==1){
                continue;
            }
            Printed[20]=1;
                sprintf(Clients[ClientCount].Response, "Enabling Measurement\n");
                Clients[ClientCount].DenyInput = 0;
        }else if (strcmp(Command,APP_COMMAND_STRING_ENABLE_CONTROL_PHONE)==0){
                
            if(Printed[21]==1){
                continue;
            }
            Printed[21]=1;
                Clients[ClientCount].Previous_Check_Time=millis()-1000;
                sprintf(Clients[ClientCount].Response, "Enabling Control Phone\n");
                Clients[ClientCount].DenyInput = 2;
        }else if (strcmp(Command,APP_COMMAND_STRING_SET_SENSORS_USED)==0){
                
            if(Printed[22]==1){
                continue;
            }
            Printed[22]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                int new_sensor_setting=0;
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                i += strlen(Parameters[0]) + 1;
                sscanf(Parameters[0],"%d",&new_sensor_setting);
                if(new_sensor_setting==0){
                    sprintf(Clients[ClientCount].Response, "Cannot disable all sensors\n");
                }else if(new_sensor_setting==Sensors_Used){
                    sprintf(Clients[ClientCount].Response, "Sensor setting unchanged\n");
                }else {
                    sprintf(Clients[ClientCount].Response, "Changing sensor setting to %d\n",new_sensor_setting);
                    Enable_Input_Lines(new_sensor_setting);
                }
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_GET_SENSORS_USED)==0){
                
            if(Printed[23]==1){
                continue;
            }
            Printed[23]=1;
            sprintf(Clients[ClientCount].Response, "Sensors_Used=%d\n", Sensors_Used);
        }else if (strcmp(Command,APP_COMMAND_STRING_GET_RAMP_TIME)==0){
                
            if(Printed[24]==1){
                continue;
            }
            Printed[24]=1;
            if(GoldRush_Mode==GOLDRUSH_SERVER_MODE_RAMP){
                sprintf(Clients[ClientCount].Response, "Ramp will end in %ld ms\n", Ramp_End_Time-millis());
            }else{
                sprintf(Clients[ClientCount].Response, "Ramp_Time=%ld\n", Ramp_Time);
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_SET_RAMP_TIME)==0){
                
            if(Printed[25]==1){
                continue;
            }
            Printed[25]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                long new_ramp_time;
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                i += strlen(Parameters[0]) + 1;
                sscanf(Parameters[0],"%d",&new_ramp_time);
                if(new_ramp_time<0){
                    sprintf(Clients[ClientCount].Response, "Ramp time cannot be negative %ld\n",new_ramp_time);
                }else{
                    sprintf(Clients[ClientCount].Response, "Setting Ramp time to %ld\n",new_ramp_time);
                    Ramp_Time=new_ramp_time;
                    StoreRampEndSettings();
                }
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_SET_RAMP_END_DISTANCE)==0){
                
            if(Printed[26]==1){
                continue;
            }
            Printed[26]=1;
            if (i <= strlen(Clients[ClientCount].Command)) {
                long new_ramp_time;
                sscanf(Clients[ClientCount].Command + i, "%s", Parameters[0]);
                i += strlen(Parameters[0]) + 1;
                sscanf(Parameters[0],"%d",&new_ramp_time);
                if(new_ramp_time<0){
                    sprintf(Clients[ClientCount].Response, "Ramp end distance cannot be negative %ld\n",new_ramp_time);
                }else{
                    sprintf(Clients[ClientCount].Response, "Setting Ramp End distance to %ld\n",new_ramp_time);
                    Ramp_End_Distance=new_ramp_time;
                    StoreRampEndSettings();
                }
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_GET_RAMP_END_DISTANCE)==0){
                
            if(Printed[27]==1){
                continue;
            }
            Printed[27]=1;
            sprintf(Clients[ClientCount].Response, "Ramp End Distance=%f\n", Ramp_End_Distance);
        }else if (strcmp(Command,APP_COMMAND_STRING_PAUSE_RAMP )==0){
                
            if(Printed[28]==1){
                continue;
            }
            Printed[28]=1;
            switch(GoldRush_Mode){
                case GOLDRUSH_SERVER_MODE_STOPPED:
                sprintf(Clients[ClientCount].Response, "Cannot pause, ride not started");
                break;
                case GOLDRUSH_SERVER_MODE_PAUSED:
                sprintf(Clients[ClientCount].Response, "System already paused");
                break;
                case GOLDRUSH_SERVER_MODE_RAMP:
                    GoldRush_Mode=GOLDRUSH_SERVER_MODE_PAUSED;
                    Ramp_End_Time=Ramp_End_Time-millis();
                sprintf(Clients[ClientCount].Response, "Pausing system");
                break;
                case GOLDRUSH_SERVER_MODE_MAIN:
                sprintf(Clients[ClientCount].Response, "Cannot pause, ride has left");
                break;
            }
        }else if (strcmp(Command,APP_COMMAND_STRING_START_RAMP )==0){
                
            if(Printed[29]==1){
                continue;
            }
            Printed[29]=1;
            switch(GoldRush_Mode){
                case GOLDRUSH_SERVER_MODE_STOPPED:
                sprintf(Clients[ClientCount].Response, "Cannot unpause , ride not started");
                break;
                case GOLDRUSH_SERVER_MODE_PAUSED:
                GoldRush_Mode=GOLDRUSH_SERVER_MODE_RAMP; 
                Ramp_End_Time=Ramp_End_Time+millis();
                sprintf(Clients[ClientCount].Response, "Unpausing system");
                break;
                case GOLDRUSH_SERVER_MODE_RAMP:
                sprintf(Clients[ClientCount].Response, "System already unpaused");
                break;
                case GOLDRUSH_SERVER_MODE_MAIN:
                sprintf(Clients[ClientCount].Response, "Cannot pause, ride has left");
                break;
            }
        }
        Clients[ClientCount].Response += strlen(Clients[ClientCount].Response);
        if(strlen(Clients[ClientCount].Response)>1000){
            i=strlen(Clients[ClientCount].Command);
        }
    }
    Clients[ClientCount].Response = Clients[ClientCount].ResponseBuffer;
}
