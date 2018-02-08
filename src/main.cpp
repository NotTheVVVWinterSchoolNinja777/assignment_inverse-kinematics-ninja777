// Assignment on Inverse Kinematics.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class Controller : public RFModule
{
    double link_length;

    BufferedPort<Vector> portMotors;
    BufferedPort<Vector> portEncoders;
    BufferedPort<Vector> portTarget;

    Vector encoders;
    Vector target;

    // compute the 3D position of the tip of the manipulator
    Vector forward_kinematics(const Vector &j) const
    {
        Vector ee(2);
        ee[0]=link_length*(cos(j[0])+cos(j[0]+j[1]) +cos(j[0]+j[1]+j[2]));
        ee[1]=link_length*(sin(j[0])+sin(j[0]+j[1])+sin(j[0]+j[1]+j[2]));
//        ee[3]=j[0]+j[1]+j[3];
        return ee;
    }

    // compute the Jacobian of the tip's position
    Matrix jacobian(const Vector &j) const
    {
        Matrix J(2,3);
        J(0,0)=-link_length*(sin(j[0])+sin(j[0]+j[1])+sin(j[0]+j[1]+j[2]));
        J(0,1)=-link_length*(sin(j[0]+j[1]) +sin(j[0]+j[1]+j[3]));
        J(0,2)= -link_length*(sin(j[0]+j[1]+j[2]));

        J(1,0)=link_length*(cos(j[0])+cos(j[0]+j[1])+cos(j[0]+j[1]+j[2]));
        J(1,1)=link_length*(cos(j[0]+j[1])+cos(j[0]+j[1]+j[2]));
        J(0,2)=link_length*(cos(j[0]+j[1]+j[2]));

        return J;
    }

public:
    bool configure(ResourceFinder &rf)override
    {
        link_length=rf.check("link-length",Value(60.0)).asDouble();

        portMotors.open("/assignment_inverse-kinematics/motors:o");
        portEncoders.open("/assignment_inverse-kinematics/encoders:i");
        portTarget.open("/assignment_inverse-kinematics/target:i");

        // init the encoder values of the 3 joints
        encoders=zeros(3);

        // init the target
        target=zeros(3);

        return true;
    }

    bool close()override
    {
        portMotors.close();
        portEncoders.close();
        portTarget.close();

        return true;
    }

    double getPeriod()override
    {
        return 0.01;
    }

    bool updateModule()override
    {
        // update the target from the net
        if (Vector *tar=portTarget.read(false))
            target=*tar;

        // update the encoder readouts from the net
        if (Vector *enc=portEncoders.read(false))
            encoders=*enc;

        // retrieve the current target position
        Vector ee_d=target.subVector(0,1);
        
        // retrieve the current target orientation
        double phi_d=target[2];


        Vector &vel=portMotors.prepare();
        vel=zeros(3);

        Vector ee=forward_kinematics(encoders);
        Matrix J=jacobian(encoders);
        Vector err=ee_d-ee;

//        Vector ee_full=zeros(3);
//        ee_full[0]=err[0];
//        ee_full[1]=err[1];
//        ee_full[2]=phi_d - encoders[0]+encoders[1]+encoders[2];

        double k=10.0;
        Matrix G=2.0*eye(2,2);
        vel=J.transposed()*pinv(J*J.transposed()+k*k*eye(2,2))*G*err;

//        yInfo()<<
//        vel = pinv(J)*err + (eye(3)-pinv(J)*J)*ee_full;
        // deliver the computed velocities to the actuators
        portMotors.writeStrict();
        
        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Controller controller;
    return controller.runModule(rf);
}
