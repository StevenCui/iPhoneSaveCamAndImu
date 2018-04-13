//
//  ViewController.m
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//  Changed to save camera image and imu only by H.K on 2018/4/13

#import "ViewController.h"
//#import "utility.hpp"
#import "CameraUtils.h"

@interface ViewController ()
@property (weak, nonatomic) IBOutlet UILabel *X_label;
@property (weak, nonatomic) IBOutlet UILabel *Y_label;
@property (weak, nonatomic) IBOutlet UILabel *Z_label;
@property (weak, nonatomic) IBOutlet UILabel *buf_label;
@property (weak, nonatomic) IBOutlet UILabel *total_odom_label;
@property (weak, nonatomic) IBOutlet UILabel *loop_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label;
@property (weak, nonatomic) IBOutlet UILabel *feature_label2;
@property (weak, nonatomic) IBOutlet UILabel *feature_label3;
@property (weak, nonatomic) IBOutlet UISlider *fovSlider;
@property (weak, nonatomic) IBOutlet UILabel *fovLabel;
@end

@implementation ViewController

/*************************** Save data for debug ***************************/

bool start_record = true;

unsigned long imageDataIndex = 0;

unsigned long imageDataReadIndex = 0;

unsigned long imuDataIndex = 0;

unsigned long imuDataReadIndex = 0;

unsigned long vinsDataIndex = 0;

unsigned long vinsDataReadIndex = 0;

queue<IMG_DATA> imgDataBuf;

NSMutableData *imuDataBuf = [[NSMutableData alloc] init];

NSData *imuReader;

NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];

NSData *vinsReader;

IMG_DATA imgData;

IMU_MSG imuData;

//KEYFRAME_DATA vinsData;

/*************************** Save data for debug ***************************/

/******************************* UI CONFIG *******************************/

// false:  VINS trajectory is the main view, AR image is in left bottom
// true: AR image is the main view, VINS is in left bottom
bool ui_main = false;

bool box_in_AR = false;

bool box_in_trajectory = false;

// If initialized finished, start show is true
bool start_show = false;

// Indicate the initialization progress rate
UIActivityIndicatorView *indicator;

// Used for show VINS trajectory and AR
@synthesize imageView;

// Used for show initialization UI
@synthesize featureImageView;

@synthesize videoCamera;

// Used for show alert if vocabulary is not ready
UIAlertView *alertView;

// Textview for showing vins status
int loop_old_index = -1;

float x_view_last = -5000;

float y_view_last = -5000;

float z_view_last = -5000;

float total_odom = 0;

/******************************* UI CONFIG *******************************/
// Store the fesature data processed by featuretracker
queue<ImgConstPtr> img_msg_buf;

// Store the IMU data for vins
queue<ImuConstPtr> imu_msg_buf;

// Store the IMU data for motion-only vins
//queue<IMU_MSG_LOCAL> local_imu_msg_buf;

// The number of measurements waiting to be processed
int waiting_lists = 0;

int frame_cnt = 0;

// Lock the feature and imu data buffer
std::mutex m_buf;

std::condition_variable con;

NSTimeInterval current_time = -1;

NSTimeInterval lateast_imu_time = -1;

int imu_prepare = 0;

// MotionManager for read imu data
CMMotionManager *motionManager;

// Segment the trajectory using color when re-initialize
int segmentation_index = 0;

// Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
// Set false: 10 HZ pose output and AR rendering in back-end
bool USE_PNP = false;

// Lock the solved VINS data feedback to featuretracker
std::mutex m_depth_feedback;

// Lock the IMU data feedback to featuretracker
std::mutex m_imu_feedback;

// Solved VINS feature feedback to featuretracker
//list<IMG_MSG_LOCAL> solved_features;

// Solved VINS status feedback to featuretracker
//VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
queue<pair<cv::Mat, double>> image_buf_loop;

// Lock the image_buf_loop
std::mutex m_image_buf_loop;

// Detect loop
//LoopClosure *loop_closure;

// Keyframe database
//KeyFrameDatabase keyframe_database;

// Control the loop detection frequency
int keyframe_freq = 0;

// Index the keyframe
int global_frame_cnt = 0;

// Record the checked loop frame
int loop_check_cnt = 0;

// Indicate if breif vocabulary read finish
bool voc_init_ok = false;

// Indicate the loop frame index
int old_index = -1;

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
bool cameraMode = true;

// Implied, updated by updateCameraMode()
bool imageCacheEnabled = cameraMode && !USE_PNP;


// MARK: ViewController Methods


NSString *filePath;

- (void)viewDidLoad {
    [super viewDidLoad];
    NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
    // ----------设置你想要的格式,hh与HH的区别:分别表示12小时制,24小时制
    [formatter setDateFormat:@"YYYY-MM-dd HH:mm:ss"];
    //现在时间,你可以输出来看下是什么格式
    NSDate *datenow = [NSDate date];
    //----------将nsdate按formatter格式转成nsstring
    NSString *currentTimeString = [formatter stringFromDate:datenow];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *filename=currentTimeString;
    filePath= [documentsDirectory stringByAppendingPathComponent:filename];
    NSFileManager *fileManager = [[NSFileManager alloc]init];
    bool flag= [fileManager fileExistsAtPath:filePath];
    if(!flag){
        NSLog(@"filepath is ok");
        NSString *fileContent2 = @"～～～～～～开始了~~~~~~!\r";
        NSData *fileData2 = [fileContent2 dataUsingEncoding:NSUTF8StringEncoding];
        [fileManager createFileAtPath:filePath contents:fileData2 attributes:nil];
    }
    
    /*******************************************Camera setup*******************************************/
    self.videoCamera = [[CvVideoCamera alloc]
                        initWithParentView:imageView];
    
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultAVCaptureSessionPreset =
    AVCaptureSessionPreset640x480;
#ifdef DATA_EXPORT
    self.videoCamera.defaultFPS = 1;
#else
    self.videoCamera.defaultFPS = 30;
#endif
    
    isCapturing = YES;
    
    [CameraUtils setExposureOffset: -1.0f];
    [videoCamera start];
    
    /****************************************Init all the thread****************************************/
    _condition=[[NSCondition alloc] init];
    
    saveData=[[NSThread alloc]initWithTarget:self selector:@selector(saveData) object:nil];
    [saveData setName:@"saveData"];
    [saveData start];
    [self imuStartUpdate];
}

/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
queue<IMG_DATA_CACHE> image_pool;
queue<VINS_DATA_CACHE> vins_pool;
IMG_DATA_CACHE image_data_cache;
cv::Mat lateast_equa;
UIImage *lateast_image;


- (void)processImage:(cv::Mat&)image
{
   float lowPart = image.at<float>(0,0);  //modify opencv library, timestamp was stored at index 0,0
    float highPart = image.at<float>(0,1);
    shared_ptr<IMG_MSG> img_msg(new IMG_MSG());
    img_msg->header = [[NSProcessInfo processInfo] systemUptime];
    float Group[2];
    Group[0] = lowPart;
    Group[1] = highPart;
    double* time_now_decode = (double*)Group;
    double time_stamp = *time_now_decode;
    
//  cv::cvtColor(image, image, CV_BGRA2RGB);
    cv::flip(image,image,-1);
    img_msg->header = time_stamp;
    
    //for save data
    if(start_record)
    {
        imgData.header = img_msg->header;
        Mat imgSave;
        cvtColor(image, imgSave, CV_BGRA2RGB);
        imgData.image = MatToUIImage(imgSave);
        imgDataBuf.push(imgData);
        NSLog(@"camera image timestamp : %f img size: %d %d imgDataBuf size is %d",time_stamp, imgSave.cols,imgSave.rows, static_cast<int>(imgDataBuf.size()));
    }
}



/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration
 
 */
double GRAVITY = 9.805;
bool imuDataFinished = false;
bool vinsDataFinished = false;
shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
vector<IMU_MSG> gyro_buf;  // for Interpolation
- (void)imuStartUpdate
{
    NSLog(@"imuStartUpdate");
    CMMotionManager *motionManager = [[CMMotionManager alloc] init];
    if (!motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    motionManager.accelerometerUpdateInterval = 0.1;
    motionManager.gyroUpdateInterval = 0.1;
#else
    motionManager.accelerometerUpdateInterval = 0.01;
    motionManager.gyroUpdateInterval = 0.01;
#endif
    
    [motionManager startDeviceMotionUpdates];
    
    [motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                        withHandler:^(CMAccelerometerData *latestAcc, NSError *error)
     {
         double header = motionManager.deviceMotion.timestamp;
         motionManager.deviceMotion.attitude.roll * 180.0 / M_PI,  //pitch for vins
         motionManager.deviceMotion.attitude.pitch * 180.0 / M_PI;  //roll for vins
         if(imu_prepare<10)
         {
             imu_prepare++;
         }
         shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
         acc_msg->header = latestAcc.timestamp;
         acc_msg->acc << -latestAcc.acceleration.x * GRAVITY,
         -latestAcc.acceleration.y * GRAVITY,
         -latestAcc.acceleration.z * GRAVITY;
         cur_acc = acc_msg;
         printf("imu acc update %lf %lf %lf %lf\n", acc_msg->header, acc_msg->acc.x(), acc_msg->acc.y(), acc_msg->acc.z());
         
     }];
    [motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error)
     {
         //The time stamp is the amount of time in seconds since the device booted.
         NSTimeInterval header = latestGyro.timestamp;
         if(header<=0)
             return;
         if(imu_prepare < 10)
             return;
         
         IMU_MSG gyro_msg;
         gyro_msg.header = header;
         gyro_msg.gyr << latestGyro.rotationRate.x,
         latestGyro.rotationRate.y,
         latestGyro.rotationRate.z;
         
         if(gyro_buf.size() == 0)
         {
             gyro_buf.push_back(gyro_msg);
             gyro_buf.push_back(gyro_msg);
             return;
         }
         else
         {
             gyro_buf[0] = gyro_buf[1];
             gyro_buf[1] = gyro_msg;
         }
         //interpolation
         shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
         if(cur_acc->header >= gyro_buf[0].header && cur_acc->header < gyro_buf[1].header)
         {
             imu_msg->header = cur_acc->header;
             imu_msg->acc = cur_acc->acc;
             imu_msg->gyr = gyro_buf[0].gyr + (cur_acc->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
             printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
             printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
         }
         else
         {
             printf("imu error %lf %lf %lf\n", gyro_buf[0].header, cur_acc->header, gyro_buf[1].header);
             return;
         }
         //for save data
         
         if(start_record)
         {
             
             NSFileHandle  *outFile;
             NSData *buffer;
             
             outFile = [NSFileHandle fileHandleForWritingAtPath:filePath];
             
             if(outFile == nil)
             {
                 NSLog(@"Open of file for writing failed");
             }
             
             //找到并定位到outFile的末尾位置(在此后追加文件)
             [outFile seekToEndOfFile];
             
             //读取inFile并且将其内容写到outFile中
             NSString *bs1 = [NSString stringWithFormat:@"%.0f,",((imu_msg->header) * 1e9)];
             NSString *bs2 = [NSString stringWithFormat:@"%lf",imu_msg->acc.x()];
             NSString *bs3 = [NSString stringWithFormat:@"%lf",imu_msg->acc.y()];
             NSString *bs4 = [NSString stringWithFormat:@"%lf",imu_msg->acc.z()];
             NSString *bs5 = [NSString stringWithFormat:@"%lf",imu_msg->gyr.x()];
             NSString *bs6 = [NSString stringWithFormat:@"%lf",imu_msg->gyr.y()];
             NSString *bs7 = [NSString stringWithFormat:@"%lf\r",  imu_msg->gyr.z()];
             NSLog(@"save imu string: %@",[NSString stringWithFormat:@"%lf",imu_msg->header * 1e9]);
             NSLog(@"save imu string: %@,%@,%@,%@,%@,%@,%@",bs1,bs2,bs3,bs4,bs5,bs6,bs7);
             
             NSString *bs = [bs1 stringByAppendingFormat:@"%@,%@,%@,%@,%@,%@",bs2,bs3,bs4,bs5,bs6,bs7];
             buffer = [bs dataUsingEncoding:NSUTF8StringEncoding];
             
             [outFile writeData:buffer];
         }
         if(start_record && 0)
         {
             //TS(record_imu_buf);
             imuData.header = imu_msg->header;
             imuData.acc = imu_msg->acc;
             imuData.gyr = imu_msg->gyr;
             [imuDataBuf appendBytes:&imuData length:sizeof(imuData)];
             imuDataIndex++;
             //TE(record_imu_buf);
             NSLog(@"record: imu %lf, %lu",imuData.header,imuDataIndex);
         }
         
         lateast_imu_time = imu_msg->header;
         
         
     }];
}


-(void)showOutputImage:(UIImage*)image
{
    [featureImageView setImage:image];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)saveData{
    while (![[NSThread currentThread] isCancelled])
    {
        @autoreleasepool
        {
            if(!imgDataBuf.empty())
            {
                IMG_DATA tmp_data;
                tmp_data = imgDataBuf.front();
                imgDataBuf.pop();
                [self recordImageTime:tmp_data];
                [self recordImage:tmp_data];
                imageDataIndex++;
                NSLog(@"save camera image: %lf %lu",tmp_data.header,imageDataIndex);
            }
        }
        [NSThread sleepForTimeInterval:0.04];
    }
}

- (void)tapSaveImageToIphone:(UIImage*)image
{
    UIImageWriteToSavedPhotosAlbum(image, self, @selector(image:didFinishSavingWithError:contextInfo:), nil);
}

- (void)image:(UIImage *)image didFinishSavingWithError:(NSError *)error contextInfo:(void *)contextInfo{
    
    if (error == nil) {
        NSLog(@"save access");
    }else{
        NSLog(@"save failed");
    }
}

- (void)checkDirectoryPath:(unsigned long)index withObject:(NSString*)directoryPath
{
    //delete already exist directory first time
    NSError *error;
    if (index == 0 && [[NSFileManager defaultManager] fileExistsAtPath:directoryPath])	//Does directory exist?
    {
        if (![[NSFileManager defaultManager] removeItemAtPath:directoryPath error:&error])	//Delete it
        {
            NSLog(@"Delete directory error: %@", error);
        }
    }
    
    //creat file directory if it does not exist
    if (![[NSFileManager defaultManager] fileExistsAtPath:directoryPath])
    {
        NSLog(@"directory does not exist");
        if (![[NSFileManager defaultManager] createDirectoryAtPath:directoryPath
                                       withIntermediateDirectories:NO
                                                        attributes:nil
                                                             error:&error])
        {
            NSLog(@"Create directory error: %@", error);
        }
    }
}

- (void)recordImu
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"IMU"]; //Add the file name
    
    [imuDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordVins
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [paths objectAtIndex:0];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:@"VINS"]; //Add the file name
    
    [vinsDataBuf writeToFile:filePath atomically:YES];
    //[msgData writeToFile:filePath atomically:YES];
}

- (void)recordImageTime:(IMG_DATA&)image_data
{
    double time = image_data.header;
    NSData *msgData = [NSData dataWithBytes:&time length:sizeof(time)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

- (void)recordImage:(IMG_DATA&)image_data
{
    NSData *msgData = UIImagePNGRepresentation(image_data.image);
    //NSData *msgData = [NSData dataWithBytes:&image_data length:sizeof(image_data)];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"];; //Get the docs directory
    
    [self checkDirectoryPath:imageDataIndex withObject:documentsPath];
    
    NSString *filename = [NSString stringWithFormat:@"%lu", imageDataIndex];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    [msgData writeToFile:filePath atomically:YES];
}

-(bool)readImageTime:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE_TIME"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *file1 = [[NSData alloc] initWithContentsOfFile:filePath];
        if (file1)
        {
            double time;
            [file1 getBytes:&time length:sizeof(time)];
            imgData.header = time;
        }
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}

-(bool)readImage:(unsigned long)index
{
    bool file_exist;
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsPath = [[paths objectAtIndex:0] stringByAppendingPathComponent:@"IMAGE"]; //Get the docs directory
    NSString *filename = [NSString stringWithFormat:@"%lu", index];
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename]; //Add the file name
    
    //check file exists
    if ([[NSFileManager defaultManager] fileExistsAtPath:filePath])
    {
        NSData *pngData = [NSData dataWithContentsOfFile:filePath];
        imgData.image = [UIImage imageWithData:pngData];
        file_exist = true;
    }
    else
    {
        file_exist = false;
        //NSLog(@"File does not exist");
    }
    return file_exist;
}



@end
