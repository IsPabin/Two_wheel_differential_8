/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Abin                                                      */
/*    Created:      2023/10/7 15:55:10                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "math.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
double g_lefttgtdeg;
double g_righttgtdeg;
// define your global instances of motors and other devices here
double min(double a, double b)
{
    if (a < b)
    {
        return a;
    }
    else
    {
        return b;
    }
}
/**
 * @brief 两轮差速模型
 *
 * 以一定的参数进行差速行走，实现转弯和画弧
 *
 * @param leftMT(motor) 左电机对象
 * @param rightMT(motor) 右电机对象
 * @param velocity(double) 车驱动轮轴中心速度（rpm）
 * @param omega(int) 车驱动轮轴角速度(rad/s)
 * @param angle(int) 旋转角度，正负决定旋转方向 >0 : 逆时针 <0 顺时针 缺省值: 0 一直转，方向由速度velocity决定
 *
 * @return (int)-1 ：后向旋转一定角度 0:一直转  1 ：前向旋转一定角度
 */
int motorDiffModelPos(motor leftMT, motor rightMT, double velocity, int omega, int angle = 0)
{
    if (angle != 0)
        velocity = abs(velocity);
    static float d = 0.38;      // 单位米
    static float wheelR = 0.04; // 单位米
    static float wheelC = wheelR * 2 * M_PI;
    double leftspeed, rightspeed;
    leftspeed = velocity + omega * d / 2;
    rightspeed = velocity - omega * d/ 2;
    Brain.Screen.printAt(10, 140, "right speed : %f", rightspeed);
    Brain.Screen.printAt(10, 160, "left speed : %f", leftspeed);
    if (angle == 0) // 没有目标角度
    {
        leftMT.spin(vex::directionType::fwd, leftspeed, vex::velocityUnits::rpm);
        rightMT.spin(vex::directionType::rev, rightspeed, vex::velocityUnits::rpm);
        return 0;
    }
    float InturnR = (min(abs(leftspeed), abs(rightspeed)) / omega)<0?-(min(abs(leftspeed), abs(rightspeed)) / omega):(min(abs(leftspeed), abs(rightspeed)) / omega);
    float InWheeld = InturnR * M_PI * angle / 180;
    float OuWheeld = (InturnR + d) * M_PI * angle / 180;
    Brain.Screen.printAt(10, 220, "InturnR: %f",InturnR);
    Brain.Screen.printAt(10, 200, "Inwheeld: %f Ouwheeld: %f",InWheeld,OuWheeld);
    if (angle < 0) // 有后退角度目标
    {
        if (abs(leftspeed) > abs(rightspeed)) // 右后转
        {
            leftMT.spinFor(vex::directionType::fwd, (double)((OuWheeld / wheelC) * 360), deg, (double)leftspeed, vex::velocityUnits::rpm, false);
            rightMT.spinFor(vex::directionType::rev, (double)((InWheeld / wheelC) * 360), deg, (double)rightspeed, vex::velocityUnits::rpm, false);
            g_lefttgtdeg=(double)((OuWheeld / wheelC) * 360);
            g_righttgtdeg=(double)((InWheeld / wheelC) * 360);
        }
        else // 左后转
        {
            leftMT.spinFor(vex::directionType::fwd, (double)((InWheeld / wheelC) * 360), deg, (double)leftspeed, vex::velocityUnits::rpm, false);
            rightMT.spinFor(vex::directionType::rev, (double)((OuWheeld / wheelC) * 360), deg, (double)rightspeed, vex::velocityUnits::rpm, false);
            g_lefttgtdeg=(double)((InWheeld / wheelC) * 360);
            g_righttgtdeg=(double)((OuWheeld / wheelC) * 360);
        }
        return -1;
    }
    else // 有前进角度目标
    {
        if (abs(leftspeed) > abs(rightspeed)) // 右前转
        {
            leftMT.spinFor(vex::directionType::fwd, (double)((OuWheeld / wheelC) * 360), deg, (double)leftspeed, vex::velocityUnits::rpm, false);
            rightMT.spinFor(vex::directionType::rev, (double)((InWheeld / wheelC) * 360), deg, (double)rightspeed, vex::velocityUnits::rpm, false);
            g_lefttgtdeg=(double)((OuWheeld / wheelC) * 360);
            g_righttgtdeg=(double)((InWheeld / wheelC) * 360);
        }
        else // 左后转
        {
            leftMT.spinFor(vex::directionType::fwd, (double)((InWheeld / wheelC) * 360), deg, (double)leftspeed, vex::velocityUnits::rpm, false);
            rightMT.spinFor(vex::directionType::rev, (double)((OuWheeld / wheelC) * 360), deg, (double)rightspeed, vex::velocityUnits::rpm, false);
            g_lefttgtdeg=(double)((InWheeld / wheelC) * 360);
            g_righttgtdeg=(double)((OuWheeld / wheelC) * 360);
            Brain.Screen.printAt(10, 200, "Inwheeld: %f",InWheeld);
        }
        return 1;
    }
}
/**
 * @brief 闭环8字运动模型
 *
 * 以一定的参数进行绕8字运动
 *
 * @param leftMT(motor) 左电机对象
 * @param rightMT(motor) 右电机对象
 * @param speed(int) 车驱动轮轴上画八字点的速度（rpm）
 * @param omega(int) 车驱动轮轴角速度(rad/s)
 * @param length(int) 8字长度(m)
 * @param width(int) 8字长度(m)
 * @param times(int) 环绕次数，缺省值：1
 * @param times(int) 环绕次数，缺省值：1
 * @param angle(int) 旋转角度，正负决定旋转方向 >0 : 逆时针 <0 顺时针 缺省值: 0 一直转，方向由速度velocity决定
 * @param eccentricity(int) 离线度，驱动轴上画8的点离中点的距离，缺省值 ： 0
 * 
 * @return (bool) 阻塞函数，完成后返回true
 */
bool eightwalk(motor leftMT, motor rightMT, int speed, int length, int width, int times = 1, int eccentricity = 0)
{
    if (times <= 0)
        return true;
    motorDiffModelPos(leftMT, rightMT, (speed * (width / 2 - eccentricity)) / (width / 2), speed/ (width / 2), 360);
    motorDiffModelPos(leftMT, rightMT, speed, 2*speed, -360);
    while (leftMT.isSpinning() || rightMT.isSpinning())
    {
        Brain.Screen.printAt(10, 40, "First lap is running!!");
        Brain.Screen.printAt(10, 60, "left-tgtdeg: %f",g_lefttgtdeg);
        Brain.Screen.printAt(10, 80, "left-nowdeg: %f",leftMT.position(deg));
        Brain.Screen.printAt(10, 100, "right-tgtdeg: %f",g_righttgtdeg);
        Brain.Screen.printAt(10, 120, "right-nowdeg: %f",rightMT.position(deg));
    }
    motorDiffModelPos(leftMT, rightMT, (speed * (width / 2 - eccentricity)) / (width / 2), speed/ (width / 2), 360);
    motorDiffModelPos(leftMT, rightMT, speed, -2*speed, -360);
    while (leftMT.isSpinning() || rightMT.isSpinning())
    {
        Brain.Screen.printAt(10, 40, "Second lap is running!!");
        Brain.Screen.printAt(10, 60, "left-tgtdeg: %f",g_lefttgtdeg);
        Brain.Screen.printAt(10, 80, "left-nowdeg: %f",leftMT.position(deg));
        Brain.Screen.printAt(10, 100, "right-tgtdeg: %f",g_righttgtdeg);
        Brain.Screen.printAt(10, 120, "right-nowdeg: %f",rightMT.position(deg));
    }
    return eightwalk(leftMT, rightMT, speed, length, width, --times, eccentricity);
}
int main()
{
    // 例化左右电机
    vex::motor rightMT(0);
    vex::motor leftMT(9);
    // 八字行走
    vex::task::sleep(10000);
    Brain.Screen.printAt(10, 20, "Task started!!");
    eightwalk(leftMT, rightMT, 50, 2, 1, 3, 0.19);
    Brain.Screen.printAt(10, 20, "Task completed!!");
    vex::task::sleep(2000);
    return 0;
}
