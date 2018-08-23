#include "walkengine.h"
#include <iostream>

#define SWITCH(x,y,z) if(x==y) x=z; else if(x==z) x=y;

#define COM_SSP 1
#define COM_DSP 0
#define LEG_RIGHT 1
#define lEG_LEFT  0

#define SIDE_STEP_FACTOR 1/2.75
#define DEBUG_WALK
//#define DEBUG_TEXT
#define DEBUG_WALK_PARAM
#define DEBUG_STOP

#define MAX_STEP 1000

WalkEngine::WalkEngine()
{

}

WalkEngine::WalkEngine(walk_parameter wp, double com_offset,
                       double px0, double py0, double heading,
                       double max_x, double max_y, double max_angle,
                       double z_sc,
                       double *x_tr, double *y_tr, double *z_tr,
                       double *t_tr, int ndata){
    /*********************************************************************
     * intialize variable
     * ******************************************************************/
    xi=0; yi=0; vxi=0; vyi=0;
    xt=0; yt=0; vxt=0; vyt=0;
    xd=0; yd=0; vxd=0; vyd=0;
    x=0; y=0, vx=0, vy=0;
    startPosX = 0; startPosY = 0;
    lastGoalX = 0; lastGoalY = 0;
    /********************************************************************/
    a = 500; b=1;
    Ti=0;
    Time=0;
    walkParameter=wp;
    Tssp = (1-walkParameter.alpha)*walkParameter.Tstride;
    Tdsp = walkParameter.alpha*walkParameter.Tstride;
    Tc = sqrt(walkParameter.Zc/9.8);
    /*********************************************************************
     * define maximum stride on x and y direction, also maximum angle step
     * initialize vector for walking parameter
     * *******************************************************************/
    mxStride = max_x;
    myStride = max_y;
    mAngle = max_angle;
    /*********************************************************************
     * reserve variable for footstep
     * *******************************************************************/
    sx = new QVector<double>(0);
    sy = new QVector<double>(0);
    steta = new QVector<double>(0);
    sxn = new double[MAX_STEP];
    syn = new double[MAX_STEP];
    stetan = new double[MAX_STEP];
    for(int i=0; i<MAX_STEP; i++){
        sxn[i] = 0;
        syn[i] = 0;
        stetan[i] = 0;
    }
    /**************************************************************************
     * initialize com, supportLeg and other leg, also legs on local coordinates
     * first support on right leg
     * note : initial acceleration on double support phase
     * ************************************************************************/
    //    cPos.pos.set(px0,py0);
    //    cPos.heading=0.0;
    Vector2 lleg_vector = RotMat2(cPos.heading)*Vector2(0,myStride/2);
    Vector2 rleg_vector = RotMat2(cPos.heading)*Vector2(0,-myStride/2);
    lLeg.pos = lleg_vector+cPos.pos;
    rLeg.pos = rleg_vector+cPos.pos;
    lLeg.heading = cPos.heading;
    rLeg.heading = cPos.heading;
    ////////////////////////////////////////////////////////////////////////////
    comPosition.pos.set(px0,py0,walkParameter.Zc-comOffset);
    comPosition.heading=0.0;
    supportLeg.leg = right_leg;
    supportLeg.isDSP = true;
    supportLeg.pos.set(px0,py0-myStride/2,0);
    rightLeg.pos = supportLeg.pos;
    leftLeg.pos.set(px0,py0+myStride/2,0);
    /******************************************
     * calculate lipm constant
     * ****************************************/
    C = cosh(walkParameter.Tstride/Tc);
    S = sinh(walkParameter.Tstride/Tc);
    D = a*pow(C-1,2)+b*pow(S/Tc,2);
    f1 = -a*(C-1)/D;
    f2 = -b*S/Tc/D;
    /**********************************************************************
     * initialize modified foot placement
     * note : modified foot placement is support leg on initial acceleration
     * *********************************************************************/
    mpx=supportLeg.pos.at(0); mpy=supportLeg.pos.at(1);
    /**********************************************************************
     * initialize swing leg spline
     * ***********************************************************************************/
    timeScale = 1/Tssp; zScale=z_sc;

    t_data = new double[ndata];
    z_data = new double[ndata];
    y_data = new double[ndata];
    x_data = new double[ndata];
    for(int i=0; i<ndata; i++){
        t_data[i] = t_tr[i];
        z_data[i] = z_tr[i];
        y_data[i] = y_tr[i];
        x_data[i] = x_tr[i];
    }
    tData = t_data;
    xTrajectory=x_data;
    yTrajectory=y_data;
    zTrajectory=z_data;

    swingLeg.trajectory_z = Spline(tData,zTrajectory,9);
    swingLeg.trajectory_y = Spline(tData,yTrajectory,9);
    swingLeg.trajectory_x = Spline(tData,xTrajectory,9);


    x_trajectory = new Spline(tData,xTrajectory,9);
    y_trajectory = new Spline(tData,yTrajectory,9);
    z_trajectory = new Spline(tData,zTrajectory,9);
    /*************************************************************************************/
    nStep = 0;
    std::cout<<"left leg : ("<<leftLeg.pos.at(0)<<","<<leftLeg.pos.at(1)<<") right leg : ("<<rightLeg.pos.at(0)<<","<<rightLeg.pos.at(1)<<")\n";

    phase0Trajectory = new Spline(tData,yTrajectory,9);
    T_phase0 = 0.5*(Tssp+Tdsp);
    comOffset = com_offset; //meter
    isWalking = false;
}

void WalkEngine::walkRequest(double x, double y, double heading){
    if(fabs(x-lastGoalX)>0 || fabs(y-lastGoalY))
        if(!isWalking){
            /******************************************************************
        * clear the vector
        * ****************************************************************/
            sx->clear();
            sy->clear();
            sx = new QVector<double>(0);
            sx = new QVector<double>(0);
            /******************************************************************
        * calculate the difference of current position (com) and desired
        * position (com)
        * ****************************************************************/
            double dx = x-comPosition.pos.at(0);
            double dy = y-comPosition.pos.at(1);
            int x_sign=1, y_sign=1;
            if(dx<0)x_sign=-1;
            if(dy<0)y_sign=-1;
            /******************************************************************
        * for the first step or initial acceleration of com, leg is on dsp
        * and support leg is right leg
        * ****************************************************************/
            supportLeg.pos = rightLeg.pos;
            supportLeg.isDSP = true;
            /******************************************************************/
#ifdef DEBUG_WALK_PARAM
            emit debugText(QString("left leg : (")+QString::number(leftLeg.pos.at(0))+QString(",")+QString::number(leftLeg.pos.at(1))+QString(")"));
            emit debugText(QString("right leg : (")+QString::number(rightLeg.pos.at(0))+QString(",")+QString::number(rightLeg.pos.at(1))+QString(")"));
            emit debugText(QString("com : (")+QString::number(comPosition.pos.at(0))+QString(",")+QString::number(comPosition.pos.at(1))+QString(")"));
            emit debugText(QString("dx : ")+QString::number(x-comPosition.pos.at(0)));
            emit debugText(QString("dy : ")+QString::number(y-comPosition.pos.at(1)));
            if(supportLeg.isDSP){
                emit debugText("suppot leg : DSP");
            }
            else {
                if(supportLeg.leg==right_leg)
                    emit debugText(QString("support leg : right leg"));
                else if(supportLeg.leg==left_leg)
                    emit debugText(QString("support leg : left leg"));
            }
#endif
            /******************************************************************
        * calculate the number of step
        * ****************************************************************/
            int x_step = (int) fabs(dx/mxStride);
            int y_step = (int) fabs(dy/(myStride*SIDE_STEP_FACTOR));
            int angle_step = (int) fabs(heading/mAngle);
            /******************************************************************
     * calculate ther remainder of step and set the walking step since
     * the number of step in the x and y direction could be different
     * ****************************************************************/
            double x_step_remaining = fabs(dx)-x_step*mxStride;
            double y_step_remaining = fabs(dy)-y_step*myStride*SIDE_STEP_FACTOR;
            double angle_remaining = heading-angle_step*mAngle;

            int walk_step = 0;
            if(x_step > y_step)
                walk_step = x_step;
            else walk_step = y_step;

#ifdef DEBUG_WALK_PARAM
            emit debugText(QString("x remaining : ")+QString::number(x_step_remaining));
            emit debugText(QString("y remaining : ")+QString::number(y_step_remaining));
#endif
            /******************************************************************
        * generating the walking parameter
        * ****************************************************************/
            int i;
            nStep = walk_step+1;
            sx->append(0); sy->append(myStride);

            for(i=1; i<=x_step; i++)
                sx->append(mxStride*x_sign);

            for(i=1; i<=y_step; i++)
                sy->append(myStride+y_sign*pow(-1,i)*myStride*SIDE_STEP_FACTOR);


            if(fabs(x_step_remaining)>0.000000 || fabs(y_step_remaining)>0.000000){
                if(fabs(x_step_remaining)>0.000000 && fabs(y_step_remaining)>0.000000){
                    sx->append(x_sign*x_step_remaining);
                    sy->append(myStride+y_sign*pow(-1,i)*y_step_remaining);
                }
                else if(fabs(x_step_remaining)==0.000000){
                    sx->append(0);
                    sy->append(myStride+y_sign*pow(-1,i)*y_step_remaining);
                }
                else if(fabs(y_step_remaining)==0.000000){
                    sx->append(x_sign*x_step_remaining);
                    sy->append(myStride);
                }
                nStep++;
            }
            for(i=x_step; i<walk_step; i++)
                sx->append(0);
            for(i=y_step; i<walk_step; i++)
                sy->append(myStride);
            sx->append(0); sy->append(myStride);
            nStep++;
            /*****************************debug********************************/
#ifdef DEBUG_WALK_PARAM
            for(i=0; i<nStep; i++)
                emit debugText(QString("stride ")+QString::number(i)+QString(" : (")+QString::number(sx->at(i))+QString(",")+QString::number(sy->at(i))+QString(")"));
#endif
            /******************************************************************/
            yp0 = -sy->at(0)/2;
            yp1 = sy->at(0)/2;
            yp2 = -(sy->at(1)-sy->at(0)/2);
            Th0 = -((yp1+yp2)/2+(C-1)*yp1)/(S*yp0);
            T0 = Tc*atanh(Th0);
            C0 = cosh(T0/Tc);
            yi0 = supportLeg.pos.at(1)+(C0-1)*yp0/C0+sy->at(0)/2;
            /******************************************************************/
            status = walk_initial_acceleration_phase0;
            /******************************************************************/
            yi=yi0; vxi=0; vyi=0;
            xi=comPosition.pos.at(0);
            mpx=supportLeg.pos.at(0);
            mpy=supportLeg.pos.at(1);
            mpy=comPosition.pos.at(1);
            /******************************debug*******************************/
#ifdef DEBUG_WALK_PARAM
            emit debugText(QString("support leg : (")+QString::number(supportLeg.pos.at(0))+QString(",")+QString::number(supportLeg.pos.at(1))+QString(")"));
            emit debugText(QString("T0 : ")+QString::number(T0)+QString(" yi0 : ")+QString::number(yi0));
            /******************************************************************/
            if(supportLeg.isDSP){
                emit supportLegChanged(leftLeg.pos.at(0),leftLeg.pos.at(1),leftLeg.pos.at(0),leftLeg.pos.at(1),lEG_LEFT);
                emit supportLegChanged(rightLeg.pos.at(0),rightLeg.pos.at(1),rightLeg.pos.at(0),rightLeg.pos.at(1),LEG_RIGHT);
            }
            else {
                if(supportLeg.leg==left_leg)
                    emit supportLegChanged(supportLeg.pos.at(0), supportLeg.pos.at(1),mpx,mpy,lEG_LEFT);
                else if(supportLeg.leg==right_leg)
                    emit supportLegChanged(supportLeg.pos.at(0), supportLeg.pos.at(1),mpx,mpy,LEG_RIGHT);
            }
            emit debugText(QString("mod. footplace : (")+QString::number(mpx)+QString(",")+QString::number(mpy)+QString(")"));
            emit debugText(QString("------------------------------------------------"));
#endif
            Ti = Time;
            currentStep = 0;
            isWalking = true;
            startPosX = comPosition.pos.at(0);
            startPosY = comPosition.pos.at(1);
            lastGoalX = x;
            lastGoalY = y;
        }
        else {
#ifdef DEBUG_WALK_PARAM
            emit debugText("============================");
#endif
            double dx, dy;
            if((nStep-currentStep)>3){

                double tempx = 0, tempy = 0;
                for(int w = 0; w<currentStep-1; w++)
                    tempx+=sx->at(w);
                for(int w = 1; w<currentStep-1; w++)
                    tempy+=(pow(-1,w)*sy->at(w)+pow(-1,w-1)*sy->at(w-1))/2;

                dx = x - (startPosX+tempx);
                dy = y - (startPosY+tempy);

#ifdef DEBUG_WALK_PARAM
                emit debugText(QString("dx : %1 dy : %2").arg(dx).arg(dy));
#endif

                if(dx>0 || dy>0){

                    if(sx->size()>1){
                        sx->remove(currentStep,sx->size()-currentStep);
                        sy->remove(currentStep,sy->size()-currentStep);
                        sx->removeLast();
                        sy->removeLast();
                    }


#ifdef DEBUG_WALK_PARAM
                    emit debugText("cut walk parameters :");
                    for(int w=0; w<sx->size(); w++){
                        emit debugText(QString("sx sy %1 : %2 %3").arg(w).arg(sx->at(w)).arg(sy->at(w)));
                    }
#endif

                    double tempx = 0, tempy=0;
                    for(int w = 0; w<sx->size(); w++)
                        tempx+=sx->at(w);
                    for(int w = 1; w<sy->size(); w++)
                        tempy+=(pow(-1,w)*sy->at(w)+pow(-1,w-1)*sy->at(w-1))/2;

                    int yStrideSign = 1;
                    if(sy->size()%2)
                        yStrideSign = -1;
                    //                dx = x - (startPosX+tempx);
                    //                dy = y - (startPosY+tempy);


#ifdef DEBUG_WALK_PARAM
                    emit debugText(QString("com.y : %1 y : %2 dy : %3").arg(comPosition.pos.at(1)).arg(y).arg(dy));
#endif

                    int x_sign=1, y_sign=1;
                    if(dx<0)
                        x_sign=-1;
                    if(dy<0)
                        y_sign=-1;

                    /******************************************************************
            * calculate the number of step
            * ****************************************************************/
                    int x_step = (int) fabs(dx/mxStride);
                    int y_step = (int) fabs(dy/(myStride*SIDE_STEP_FACTOR));
                    int angle_step = (int) fabs(heading/mAngle);
                    /******************************************************************
            * calculate ther remainder of step and set the walking step since
            * the number of step in the x and y direction could be different
            * ****************************************************************/
                    double x_step_remaining = fabs(dx)-x_step*mxStride;
                    double y_step_remaining = fabs(dy)-y_step*myStride*SIDE_STEP_FACTOR;
                    double angle_remaining = heading-angle_step*mAngle;

                    int walk_step = 0;
                    if(x_step > y_step)
                        walk_step = x_step;
                    else walk_step = y_step;

#ifdef DEBUG_WALK_PARAM
                    emit debugText(QString("currentStep : %1 walk_step : %2").arg(currentStep).arg(walk_step));
#endif
                    /******************************************************************
            * generating the walking parameter
            * ****************************************************************/

                    int i=0;
                    nStep = currentStep+2+walk_step+1;

                    for(i=0; i<x_step; i++)
                        sx->append(mxStride*x_sign);

                    for(i=0; i<y_step; i++)
                        sy->append(myStride+y_sign*pow(-1,i)*yStrideSign*myStride*SIDE_STEP_FACTOR);

                    if(fabs(x_step_remaining)>0 || fabs(y_step_remaining)>0){

#ifdef DEBUG_WALK_PARAM
                        emit debugText(QString("adding rem step : %1 %2").arg(x_step_remaining).arg(y_step_remaining));
#endif
                        if(fabs(x_step_remaining)>0 && fabs(y_step_remaining)>0){
                            sx->append(x_sign*x_step_remaining);
                            if(sy->size()%2)
                                sy->append(myStride+y_sign*pow(-1,i)*y_step_remaining);
                            else
                                sy->append(myStride+y_sign*pow(-1,i+1)*y_step_remaining);
                        }
                        else if(fabs(x_step_remaining)==0){
                            sx->append(0);
                            if(sy->size()%2)
                                sy->append(myStride+y_sign*pow(-1,i)*y_step_remaining);
                            else
                                sy->append(myStride+y_sign*pow(-1,i+1)*y_step_remaining);
                        }
                        else if(fabs(y_step_remaining)==0){
                            sx->append(x_sign*x_step_remaining);
                            sy->append(myStride);
                        }
                        nStep++;
                    }

                    for(i=x_step; i<walk_step; i++)
                        sx->append(0);
                    for(i=y_step; i<walk_step; i++)
                        sy->append(myStride);

                    sx->append(0); sy->append(myStride);
                    nStep++; nStep = sx->size();

#ifdef DEBUG_WALK_PARAM
                    emit debugText(QString("nStep : %1 array length : %2").arg(nStep).arg(sx->size()));
                    for(int i=0; i<sx->size(); i++)
                        emit debugText(QString("new stride %1 : %2 %3").arg(i).arg(sx->at(i)).arg(sy->at(i)));
#endif
                    lastGoalX = x;
                    lastGoalY = y;
                }

                else {
                    stop();
                }
            }

#ifdef DEBUG_WALK_PARAM
            emit debugText("============================");
#endif
        }
}

void WalkEngine::stop(){
    if(isWalking){
#ifdef DEBUG_STOP
        emit debugText("stopping");
#endif

        sx->remove(currentStep+2,sx->size()-(currentStep+2));
        sy->remove(currentStep+2,sy->size()-(currentStep+2));

        sx->append(0);
        sy->append(myStride);

        nStep = sx->size();
        //        sx->removeLast();
        //        sy->removeLast();
    }
}

void WalkEngine::update(double time){
    Time+=time;
    double dt;
    double l_x = comPosition.pos.at(0), l_y = comPosition.pos.at(1);

    dt = Time - Ti;

    switch(status){
    case walk_initial_acceleration_phase0:{
        if(dt<T_phase0){
            x = mpx;
            y = mpy+(yi-mpy)*phase0Trajectory->at(dt/T0);
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
#ifdef DEBUG_PHASE0
            qDebug()<<"phase 0, y :"<<y<<"mpy :"<<mpy<<"y trajectory:"<<(yi-mpy)*phase0Trajectory->at(dt/T0);
#endif
        }
        else if(dt>=T_phase0){
#ifdef DEBUG_PHASE0
            emit debugText("Phase 0");
#endif
            Ti=Time;
            yi=yi0; vxi=0; vyi=0;
            xi=comPosition.pos.at(0);
            mpx=supportLeg.pos.at(0);
            mpy=supportLeg.pos.at(1);
            status = walk_initial_acceleration;
        }
        break;
    }
    case walk_initial_acceleration:{
        if(dt<T0){
            x = (xi-mpx)*cosh(dt/Tc)+Tc*vxi*sinh(dt/Tc)+mpx;
            y = (yi-mpy)*cosh(dt/Tc)+Tc*vyi*sinh(dt/Tc)+mpy;
            vx = (xi-mpx)*sinh(dt/Tc)/Tc+vxi*cosh(dt/Tc);
            vy = (yi-mpy)*sinh(dt/Tc)/Tc+vyi*cosh(dt/Tc);
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
        }
        else if(dt>=T0){
            /******************************************************************/
#ifdef DEBUG_TEXT
            emit debugText("initial acceleration");
#endif
            status = walk_first_step;
            Ti=Time;
            dt=T0;
            x = (xi-mpx)*cosh(dt/Tc)+Tc*vxi*sinh(dt/Tc)+mpx;
            y = (yi-mpy)*cosh(dt/Tc)+Tc*vyi*sinh(dt/Tc)+mpy;
            vx = (xi-mpx)*sinh(dt/Tc)/Tc+vxi*cosh(dt/Tc);
            vy = (yi-mpy)*sinh(dt/Tc)/Tc+vyi*cosh(dt/Tc);
            xi=x; yi=y; vxi=vx; vyi=vy;
            /******************************************************************
            * define the support leg and the swing leg for the first SSP
            * first support is left leg
            * ****************************************************************/
            supportLeg.leg = left_leg;
            supportLeg.pos = leftLeg.pos;
            swingLeg.leg = right_leg;
            swingLeg.pos = rightLeg.pos;
            /*******************************************************************
            * calculate the terminal condition of com and calculate the modified
            * foot placement ro realize the terminal condition
            * *****************************************************************/
            xt=sx->at(1)/2; yt=-1*sy->at(1)/2;
            vxt=(C+1)/(Tc*S)*xt; vyt=(C-1)/(Tc*S)*yt;
            vxd=vxt; vyd=vyt;
            xd=supportLeg.pos.at(0)+xt; yd=supportLeg.pos.at(1)+yt;
            mpx=f1*(xd-C*xi-Tc*S*vxi)+f2*(vxd-S*xi/Tc-C*vxi);
            mpy=supportLeg.pos.at(1);
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
            /*******************************************************************
            * calculate the last support phase and next support phase that would
            * be used for swing leg trajectory
            * *****************************************************************/
            lastSupportX = rightLeg.pos.at(0);
            lastSupportY = rightLeg.pos.at(1);
            nextSupportX = lastSupportX+sx->at(1);
            int sign = 1;
            if((sy->at(1)-sy->at(0))<0)
                sign = -1;
            nextSupportY = lastSupportY-pow(-1,2)*(sy->at(1)-sy->at(0));
            /******************************************************************/
            emit debugText(QString("support leg : (")+QString::number(supportLeg.pos.at(0))+QString(",")+QString::number(supportLeg.pos.at(1))+QString(")"));
            emit debugText(QString("mod. foot place : (")+QString::number(mpx)+QString(",")+QString::number(mpy)+QString(")"));
            if(supportLeg.leg==left_leg)
                emit supportLegChanged(supportLeg.pos.at(0), supportLeg.pos.at(1),mpx,mpy,lEG_LEFT);
            else if(supportLeg.leg==right_leg)
                emit supportLegChanged(supportLeg.pos.at(0), supportLeg.pos.at(1),mpx,mpy,LEG_RIGHT);
            /******************************************************************/

            double xi_dsp = x, yi_dsp = y;
            double vxi_dsp = vx, vyi_dsp = vy;

            double xf_dsp = (xi_dsp-mpx)*cosh(Tdsp/2/Tc)+vxi_dsp*sinh(Tdsp/2/Tc)*Tc+mpx;
            double yf_dsp = (yi_dsp-mpy)*cosh(Tdsp/2/Tc)+vyi_dsp*sinh(Tdsp/2/Tc)*Tc+mpy;
            double vxf_dsp = (xi_dsp-mpx)*sinh(Tdsp/2/Tc)/Tc+vxi_dsp*cosh(Tdsp/2/Tc);
            double vyf_dsp = (yi_dsp-mpy)*sinh(Tdsp/2/Tc)/Tc+vyi_dsp*cosh(Tdsp/2/Tc);
            double t3=pow(Tdsp,3), t2=pow(Tdsp,2), t1=Tdsp;
            double m[16] = {1,0,0,0,
                            0,1,0,0,
                            1,t1,t2,t3,
                            0,1,2*t1,3*t2};
            double xptr[4] = {x,vx,xf_dsp,vxf_dsp};
            double yptr[4] = {y,vy,yf_dsp,vyf_dsp};
            /******************************************************************/
            /******************************************************************/
            Matrix4 Mat(m);
            Vector4 X(xptr);
            Vector4 Y(yptr);
            Vector4 ax = Mat.inverse()*X;
            Vector4 ay = Mat.inverse()*Y;
            ax0 = ax.at(0); ax1 = ax.at(1); ax2 = ax.at(2); ax3 = ax.at(3);
            ay0 = ay.at(0); ay1 = ay.at(1); ay2 = ay.at(2); ay3 = ay.at(3);
        }
    }
        break;
    case walk_first_step:{
        double t3=pow(dt,3), t2=pow(dt,2), t1=dt;
        if(dt<Tdsp){
            x=ax0+ax1*t1+ax2*t2+ax3*t3;
            y=ay0+ay1*t1+ay2*t2+ay3*t3;
            vx=ax1+2*ax2*t1+3*ax3*t2;
            vy=ay1+2*ay2*t1+3*ay3*t2;
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
        }
        else{
#ifdef DEBUG_TEXT
            emit debugText("first step");
#endif
            status = walk_ssp;
        }
    }
        break;
    case walk_ssp:{
        /******************************************************************/
        if(dt>=Tssp)
            dt=Tssp;
        x = (xi-mpx)*cosh(dt/Tc)+Tc*vxi*sinh(dt/Tc)+mpx;
        y = (yi-mpy)*cosh(dt/Tc)+Tc*vyi*sinh(dt/Tc)+mpy;
        vx = (xi-mpx)*sinh(dt/Tc)/Tc+vxi*cosh(dt/Tc);
        vy = (yi-mpy)*sinh(dt/Tc)/Tc+vyi*cosh(dt/Tc);
        comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
        double x_swingleg = (nextSupportX-lastSupportX)*swingLeg.trajectory_x.at(dt*timeScale)+lastSupportX;
        double y_swingleg = (nextSupportY-lastSupportY)*swingLeg.trajectory_y.at(dt*timeScale)+lastSupportY;
        double z_swingleg = zScale*swingLeg.trajectory_z.at(dt*timeScale);
        swingLeg.pos.set(x_swingleg,y_swingleg,z_swingleg);
        if(supportLeg.leg==left_leg){
            leftLeg.pos = supportLeg.pos;
            rightLeg.pos = swingLeg.pos;
        }
        else if(supportLeg.leg==right_leg){
            rightLeg.pos = supportLeg.pos;
            leftLeg.pos = swingLeg.pos;
        }
        /******************************************************************/
        if(dt>=Tssp){
#ifdef DEBUG_TEXT
            emit debugText("Walk SSP");
#endif
            xi=x; yi=y; vxi=vx; vyi=vy;
            status = walk_dsp_initial;
            Ti=Time;
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
        }
        /******************************************************************/
    }
        break;
    case walk_dsp_initial: {
        double xi_dsp = xd, yi_dsp = yd;
        double vxi_dsp = vxd, vyi_dsp = vyd;
        /******************************************************************/

        double temp = supportLeg.pos.at(0)+sx->at(currentStep+1);
        supportLeg.pos.set(0,temp);
        temp = supportLeg.pos.at(1)-pow(-1,currentStep)*sy->at(currentStep+1);
        supportLeg.pos.set(1,temp);
        /******************************************************************
            * switch support leg and swing leg
            * ****************************************************************/
        if(supportLeg.leg==left_leg){
            supportLeg.leg=right_leg;
            swingLeg.leg=left_leg;
            swingLeg.pos=leftLeg.pos;
        }
        else if(supportLeg.leg==right_leg){
            supportLeg.leg=left_leg;
            swingLeg.leg=right_leg;
            swingLeg.pos=rightLeg.pos;
        }
        if(currentStep == nStep-2){
            xt=0; yt=pow(-1,currentStep+2)*sy->at(nStep-1)/2;
            vxt=0; vyt=0;
        }
        else {
            xt=sx->at(currentStep+2)/2; yt=pow(-1,currentStep+2)*sy->at(currentStep+2)/2;
            vxt=(C+1)/(Tc*S)*xt; vyt=(C-1)/(Tc*S)*yt;
            /////////////////////////////////////////////////////////////////////////////
            if(supportLeg.leg==right_leg){
                lastSupportX=leftLeg.pos.at(0);
                lastSupportY=leftLeg.pos.at(1);
            }
            else if(supportLeg.leg==left_leg){
                lastSupportX=rightLeg.pos.at(0);
                lastSupportY=rightLeg.pos.at(1);
            }
            nextSupportX=supportLeg.pos.at(0)+sx->at(currentStep+2);
            nextSupportY=supportLeg.pos.at(1)-pow(-1,currentStep+1)*sy->at(currentStep+2);
        }
        vxd=vxt; vyd=vyt;
        xd=supportLeg.pos.at(0)+xt; yd=supportLeg.pos.at(1)+yt;
        /******************************************************************/
        mpx=f1*(xd-C*xi_dsp-Tc*S*vxi_dsp)+f2*(vxd-S*xi_dsp/Tc-C*vxi_dsp);
        mpy=f1*(yd-C*yi_dsp-Tc*S*vyi_dsp)+f2*(vyd-S*yi_dsp/Tc-C*vyi_dsp);
        /******************************************************************/
        double xf_dsp = (xi_dsp-mpx)*cosh(Tdsp/2/Tc)+vxi_dsp*sinh(Tdsp/2/Tc)*Tc+mpx;
        double yf_dsp = (yi_dsp-mpy)*cosh(Tdsp/2/Tc)+vyi_dsp*sinh(Tdsp/2/Tc)*Tc+mpy;
        double vxf_dsp = (xi_dsp-mpx)*sinh(Tdsp/2/Tc)/Tc+vxi_dsp*cosh(Tdsp/2/Tc);
        double vyf_dsp = (yi_dsp-mpy)*sinh(Tdsp/2/Tc)/Tc+vyi_dsp*cosh(Tdsp/2/Tc);
        double t3=pow(Tdsp,3), t2=pow(Tdsp,2), t1=Tdsp;
        double m[16] = {1,0,0,0,
                        0,1,0,0,
                        1,t1,t2,t3,
                        0,1,2*t1,3*t2};
        double xptr[4] = {x,vx,xf_dsp,vxf_dsp};
        double yptr[4] = {y,vy,yf_dsp,vyf_dsp};
        /******************************************************************/
        /******************************************************************/
        Matrix4 Mat(m);
        Vector4 X(xptr);
        Vector4 Y(yptr);
        Vector4 ax = Mat.inverse()*X;
        Vector4 ay = Mat.inverse()*Y;
        ax0 = ax.at(0); ax1 = ax.at(1); ax2 = ax.at(2); ax3 = ax.at(3);
        ay0 = ay.at(0); ay1 = ay.at(1); ay2 = ay.at(2); ay3 = ay.at(3);
        /******************************************************************/
        status = walk_dsp;
        /******************************************************************/
        if(supportLeg.leg==left_leg)
            emit supportLegChanged(supportLeg.pos.at(0),supportLeg.pos.at(1),mpx,mpy,lEG_LEFT);
        else if(supportLeg.leg==right_leg)
            emit supportLegChanged(supportLeg.pos.at(0),supportLeg.pos.at(1),mpx,mpy,LEG_RIGHT);
    }
        break;
    case walk_dsp:{
        double t3=pow(dt,3), t2=pow(dt,2), t1=dt;
        /******************************************************************/
        if(dt<Tdsp){
            x=ax0+ax1*t1+ax2*t2+ax3*t3;
            y=ay0+ay1*t1+ay2*t2+ay3*t3;
            vx=ax1+2*ax2*t1+3*ax3*t2;
            vy=ay1+2*ay2*t1+3*ay3*t2;
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
            rlegik = comPosition.pos-rightLeg.pos;
            llegik = comPosition.pos-leftLeg.pos;
        }
        /******************************************************************/
        else if(dt>=Tdsp){
#ifdef DEBUG_TEXT
            emit debugText(QString("Walk DSP : currentStep %1 nstep %2 isWalking %3").arg(currentStep).arg(nStep).arg(isWalking));
#endif
            dt = Tdsp;
            t3=pow(dt,3); t2=pow(dt,2); t1=dt;
            x=ax0+ax1*t1+ax2*t2+ax3*t3;
            y=ay0+ay1*t1+ay2*t2+ay3*t3;
            vx=ax1+2*ax2*t1+3*ax3*t2;
            vy=ay1+2*ay2*t1+3*ay3*t2;
            xi=x; yi=y; vxi=vx; vyi=vy;
            status = walk_ssp;
            Ti=Time;
            currentStep++;
            comPosition.pos.set(x,y,walkParameter.Zc-comOffset);
            rlegik = comPosition.pos-rightLeg.pos;
            llegik = comPosition.pos-leftLeg.pos;

            if(currentStep==(nStep-1)){
                emit debugText("end walking");
                isWalking = false;
                status = walk_finish;
                currentStep = 0;
                emit debugText(QString("isWalking = %1 currentStep = %2").arg(isWalking).arg(currentStep));
                comPosition.pos.set(xd,yd,walkParameter.Zc-comOffset);
            }

        }
        /******************************************************************/

        break;
    }
    case walk_finish:{
#ifdef DEBUG_TEXT
        emit debugText(QString("walk finished"));
#endif
        if(supportLeg.leg==right_leg)
            emit debugText("last support leg : right leg");
        else if(supportLeg.leg==left_leg)
            emit debugText("last support leg : left leg");
        emit debugText(QString("com : (")+QString::number(comPosition.pos.at(0))+QString(",")+QString::number(comPosition.pos.at(1))+QString(")"));
        emit debugText(QString("------------------------------------------------"));

        bool s=true;
        if(status==walk_ssp)
            s=false;
        emit comLegChanged(comPosition.pos.at(0),comPosition.pos.at(1),s,rightLeg.pos.at(0),rightLeg.pos.at(1),leftLeg.pos.at(0),leftLeg.pos.at(1),xd,yd);
        status=walk_idle;
        dt=0;
    }
        break;
    case walk_idle:
        isWalking = false;
        break;
    default:
        status = walk_idle;
        break;
    }

    if(l_x!=comPosition.pos.at(0)||l_y!=comPosition.pos.at(1)){
        bool s = false;
        if(status == walk_ssp)
            s = true;
        emit comChanged(x,y,s);
        emit comLegChanged(x,y,s,rightLeg.pos.at(0),rightLeg.pos.at(1),leftLeg.pos.at(0),leftLeg.pos.at(1),xd,yd);

#ifdef DEBUG_WALK_TRAJECTORY
        emit xlegTrajectoryChanged(Time,llegik.at(0),0);
        emit xlegTrajectoryChanged(Time,rlegik.at(0),1);
        emit ylegTrajectoryChanged(Time,llegik.at(1),0);
        emit ylegTrajectoryChanged(Time,rlegik.at(1),1);
        emit zlegTrajectoryChanged(Time,llegik.at(2),0);
        emit zlegTrajectoryChanged(Time,rlegik.at(2),1);
#endif

    }

    /****************************************************************************************
     * calculate ik leg
     * *************************************************************************************/

    Vector3 tmp = comPosition.pos;
    tmp.set(2,walkParameter.Zc-comOffset);

    llegik = tmp - leftLeg.pos;
    rlegik = tmp - rightLeg.pos;

#ifdef DEBUG_IKLEG
    qDebug()<<"ik : "<<llegik.at(0)<<","<<llegik.at(1)<<","<<llegik.at(2)<<" "<<
              rlegik.at(0)<<","<<rlegik.at(1)<<","<<rlegik.at(2);
#endif

}

double WalkEngine::getTime(){
    return Time;
}
