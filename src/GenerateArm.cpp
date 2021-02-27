#include <sdf_mp_integration/GenerateArm.h>



gpmp2::ArmModel GeneratePandaArm(const gtsam::Point3 &base_pos){

    uint dof = 7; 
    gtsam::Vector a(dof); 
    gtsam::Vector alpha(dof);
    gtsam::Vector d(dof);
    gtsam::Vector theta(dof);
    gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3(), base_pos);


    alpha << -1.5708,    1.5708,   1.5708,  -1.5708,  1.5708,  1.5708,  0;
    a     << 0,          0,        0.0825,  -0.0825,  0,        0.088,  0;
    d     << 0.333,      0,        0.316,   0,        0.384,    0,      0.107;
    theta << 0, 0, 0, 0, 0, 0, -1.5708/2;

    gpmp2::Arm abs_arm = gpmp2::Arm(dof, a, alpha, d, base_pose, theta);

    gpmp2::BodySphereVector spheres_vec = {
        // gpmp2::BodySphere(0, 0.120000, gtsam::Point3(-0.03000, 0.215000, 0.000000)),
        // gpmp2::BodySphere(0, 0.080000, gtsam::Point3(0.00000, 0.120000, 0.000000)),
        // gpmp2::BodySphere(0, 0.080000, gtsam::Point3(0.00000, 0.000000, -0.06000)),

        // gpmp2::BodySphere(1, 0.080000, gtsam::Point3(0.06000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(1, 0.080000, gtsam::Point3(0.00000, 0.000000, 0.150000)),
        // gpmp2::BodySphere(1, 0.060000, gtsam::Point3(0.00000, 0.000000, 0.250000)),

        // gpmp2::BodySphere(2, 0.120000, gtsam::Point3(0.00000, 0.000000, 0.000000)),

        // gpmp2::BodySphere(3, 0.090000, gtsam::Point3(0.00000, 0.000000, 0.120000)),

        // gpmp2::BodySphere(4, 0.080000, gtsam::Point3(0.00000, -0.13000, -0.10000)),
        // gpmp2::BodySphere(4, 0.100000, gtsam::Point3(0.00000, 0.000000, -0.04000)),

        // gpmp2::BodySphere(5, 0.080000, gtsam::Point3(0.00000, 0.000000, 0.000000)),

        // gpmp2::BodySphere(6, 0.060000, gtsam::Point3(0.00000, 0.050000, 0.010000)),
        // gpmp2::BodySphere(6, 0.030000, gtsam::Point3(0.00000, 0.050000, 0.090000)),
        // gpmp2::BodySphere(6, 0.060000, gtsam::Point3(0.00000, -0.05000, 0.010000)),
        // gpmp2::BodySphere(6, 0.030000, gtsam::Point3(0.00000, -0.05000, 0.090000))};
        
        gpmp2::BodySphere(0, 0.140000, gtsam::Point3(-0.02000, 0.235000, 0.000000)),
        gpmp2::BodySphere(0, 0.100000, gtsam::Point3(0.00000, 0.120000, 0.000000)),
        gpmp2::BodySphere(0, 0.120000, gtsam::Point3(0.00000, 0.000000, -0.06000)),
        gpmp2::BodySphere(0, 0.120000, gtsam::Point3(0.00000, 0.000000, 0.06000)),

        gpmp2::BodySphere(1, 0.120000, gtsam::Point3(0.00000, 0.000000, 0.120000)),
        gpmp2::BodySphere(1, 0.120000, gtsam::Point3(0.00000, 0.000000, 0.240000)),

        gpmp2::BodySphere(2, 0.120000, gtsam::Point3(0.00000, 0.000000, 0.050000)),
        gpmp2::BodySphere(2, 0.120000, gtsam::Point3(0.00000, 0.000000, -0.050000)),

        gpmp2::BodySphere(3, 0.100000, gtsam::Point3(0.00000, 0.000000, 0.070000)),
        gpmp2::BodySphere(3, 0.100000, gtsam::Point3(0.00000, 0.000000, 0.170000)),

        // The straight shaft bit
        gpmp2::BodySphere(4, 0.080000, gtsam::Point3(0.00000, -0.15000, -0.08000)),
        gpmp2::BodySphere(4, 0.080000, gtsam::Point3(0.00000, -0.07000, -0.10000)),
        gpmp2::BodySphere(4, 0.090000, gtsam::Point3(0.00000, 0.00000, -0.10000)),
        gpmp2::BodySphere(4, 0.110000, gtsam::Point3(0.00000, 0.000000, 0.00000)),

        gpmp2::BodySphere(5, 0.100000, gtsam::Point3(0.00000, 0.000000, -0.030000)),
        gpmp2::BodySphere(5, 0.100000, gtsam::Point3(0.00000, 0.000000, 0.070000)),

        gpmp2::BodySphere(6, 0.080000, gtsam::Point3(0.00000, 0.070000, 0.030000)),
        gpmp2::BodySphere(6, 0.080000, gtsam::Point3(0.00000, 0.00000, 0.030000)),
        gpmp2::BodySphere(6, 0.080000, gtsam::Point3(0.00000, -0.07000, 0.030000)),
        
        // The gripper 
        gpmp2::BodySphere(6, 0.050000, gtsam::Point3(0.00000, 0.05000, 0.080000)),
        gpmp2::BodySphere(6, 0.050000, gtsam::Point3(0.00000, -0.05000, 0.080000)),
        gpmp2::BodySphere(6, 0.050000, gtsam::Point3(0.00000, 0.000000, 0.080000))};


    gpmp2::ArmModel arm_model = gpmp2::ArmModel(abs_arm, spheres_vec);

    return arm_model;
};

gpmp2::Pose2MobileVetLinArmModel GenerateHSRArm(const gtsam::Point3 &base_pos){

    uint dof = 4; 
    gtsam::Vector a(dof); 
    gtsam::Vector alpha(dof);
    gtsam::Vector d(dof);
    gtsam::Vector theta(dof);
    gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3(), base_pos);


    // Kind of works
    //       To get roll,  , To get flex         to get second roll,         TO DO
    // alpha << -1.57,         1.57,             -1.57,           0;
    // a     << 0,             0,                0,                0.0;
    // d     << 0,             0.345,            0.0,              0.1;
    // theta << -1.5708,       0,                0,                0;
    
    // alpha << 0,         1.57,          -1.57,                   1.57;
    // a     << 0,             0.2 ,             0,                    0;
    // d     << 0,             0,              0.345,                  0;
    // theta << 0,             0,          0,                      0;
    // theta << 0,             -1.57,          0,                      0;



    // New attempt 1) roll 2) flex, 3) second roll
    alpha << -1.57,          1.570,             -1.57,                   0;
    a     << 0,             0.005 ,                 0,                   0;
    d     << 0,             0.345,             0,                   0;
    theta << 0,             0,                  0,                   0;
    
    gpmp2::Arm abs_arm = gpmp2::Arm(dof, a, alpha, d, base_pose, theta);

    gpmp2::BodySphereVector spheres_vec = {
        // gpmp2::BodySphere(0, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(0, 0.01000, gtsam::Point3(0.01000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(0, 0.01000, gtsam::Point3(0.00000, 0.010000, 0.000000)),
        // gpmp2::BodySphere(0, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.010000)),

        // gpmp2::BodySphere(1, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(1, 0.01000, gtsam::Point3(0.01000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(1, 0.01000, gtsam::Point3(0.00000, 0.010000, 0.000000)),
        // gpmp2::BodySphere(1, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.010000)),

        // gpmp2::BodySphere(2, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(2, 0.01000, gtsam::Point3(0.01000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(2, 0.01000, gtsam::Point3(0.00000, 0.010000, 0.000000)),
        // gpmp2::BodySphere(2, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.010000)),

        // gpmp2::BodySphere(3, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(3, 0.01000, gtsam::Point3(0.01000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(3, 0.01000, gtsam::Point3(0.00000, 0.010000, 0.000000)),
        // gpmp2::BodySphere(3, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.010000)),
    
        // gpmp2::BodySphere(4, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(4, 0.01000, gtsam::Point3(0.02000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(4, 0.01000, gtsam::Point3(0.00000, 0.040000, 0.000000)),
        // gpmp2::BodySphere(4, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.060000)),

        // gpmp2::BodySphere(5, 0.02000, gtsam::Point3(0.00000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(5, 0.01000, gtsam::Point3(0.01000, 0.000000, 0.000000)),
        // gpmp2::BodySphere(5, 0.01000, gtsam::Point3(0.00000, 0.010000, 0.000000)),
        // gpmp2::BodySphere(5, 0.01000, gtsam::Point3(0.00000, 0.000000, 0.010000))


        // gpmp2::BodySphere(0, 0.420000, gtsam::Point3( 0.0000, 0.00000,  0.230000)),

        // // Base
        gpmp2::BodySphere(0, 0.220000, gtsam::Point3( -0.14000, 0.00000,  0.130000)),
        gpmp2::BodySphere(0, 0.220000, gtsam::Point3( 0.14000, 0.00000,  0.130000)),
        gpmp2::BodySphere(0, 0.220000, gtsam::Point3( 0.00000, 0.14000,  0.130000)),
        gpmp2::BodySphere(0, 0.220000, gtsam::Point3( 0.00000, -0.14000,  0.130000)),

        gpmp2::BodySphere(0, 0.270000, gtsam::Point3( -0.07000, -0.07000,  0.350000)),
        gpmp2::BodySphere(0, 0.270000, gtsam::Point3( -0.07000, -0.07000,  0.620000)),
        gpmp2::BodySphere(0, 0.270000, gtsam::Point3( -0.07000, 0.07000,  0.350000)),
        gpmp2::BodySphere(0, 0.270000, gtsam::Point3( -0.07000, 0.07000,  0.620000)),


        // Relative to the lift
        gpmp2::BodySphere(1, 0.110000, gtsam::Point3(0.00000, 0.000000, 0.100000)),
        gpmp2::BodySphere(1, 0.110000, gtsam::Point3(0.00000, 0.000000, 0.200000)),
        gpmp2::BodySphere(1, 0.110000, gtsam::Point3(0.00000, 0.000000, 0.300000)),

        // Head
        gpmp2::BodySphere(1, 0.270000, gtsam::Point3(-0.04000, 0.000000, 0.530000)),
        
        // First arm section (just flex)
        gpmp2::BodySphere(2, 0.110000, gtsam::Point3(0.000000, 0.000000, 0.000000)),
        gpmp2::BodySphere(2, 0.110000, gtsam::Point3(0.000000, 0.000000, 0.12000)),
        gpmp2::BodySphere(2, 0.110000, gtsam::Point3(0.000000, 0.000000, 0.220000)),

        // Second arm section - twists with roll (axis is at base of first section though)
        gpmp2::BodySphere(3, 0.10000, gtsam::Point3(0.01000, 0.00000, 0.0000000)),
        gpmp2::BodySphere(3, 0.10000, gtsam::Point3(0.01000, -0.07000, 0.0000000)),

        // Wrist
        gpmp2::BodySphere(5, 0.0900, gtsam::Point3(-0.010000, 0.000000, 0.10000)),
        gpmp2::BodySphere(5, 0.0400, gtsam::Point3(0.000000, 0.04000, 0.150000)),
        gpmp2::BodySphere(5, 0.0400, gtsam::Point3(0.000000, -0.04000, 0.150000)),
        gpmp2::BodySphere(5, 0.0400, gtsam::Point3(0.000000, 0.04000, 0.190000)),
        gpmp2::BodySphere(5, 0.0400, gtsam::Point3(0.000000, -0.04000, 0.190000))
    };

    gtsam::Pose3 base_T_torso = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0.34));

    // Note that this gives the flex joint position correctly (rather than using the DH params)
    gtsam::Pose3 torso_T_arm = gtsam::Pose3(gtsam::Rot3::RzRyRx(1.57, 0, 0), gtsam::Point3(0.141, 0.078, 0));

    // abstract mobile arm
    gpmp2::Pose2MobileVetLinArm marm(abs_arm, base_T_torso, torso_T_arm, false);
    
    gpmp2::Pose2MobileVetLinArmModel model = gpmp2::Pose2MobileVetLinArmModel(marm, spheres_vec);

    return model;
};

gpmp2::Pose2MobileVetLinArmModel GenerateHSRArm(){
    return GenerateHSRArm(gtsam::Point3(0,0,0));
};

gpmp2::ArmModel GeneratePandaArm(){
    return GeneratePandaArm(gtsam::Point3(0,0,0));
};
