% Simscape(TM) Multibody(TM) version: 7.1

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(3).translation = [0.0 0.0 0.0];
smiData.RigidTransform(3).angle = 0.0;
smiData.RigidTransform(3).axis = [0.0 0.0 0.0];
smiData.RigidTransform(3).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 25.400000000000091 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(1).ID = 'B[Base-1:-:SimpleSteps-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [914.39999999999986 0 -304.80000000000086];  % mm
smiData.RigidTransform(2).angle = 1.5707963267948966;  % rad
smiData.RigidTransform(2).axis = [-1 7.8504622934188758e-17 7.8504622934188758e-17];
smiData.RigidTransform(2).ID = 'F[Base-1:-:SimpleSteps-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [1156.8681045801181 1562.0675088134217 2340.2706862313644];  % mm
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = 'RootGround[Base-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(2).mass = 0.0;
smiData.Solid(2).CoM = [0.0 0.0 0.0];
smiData.Solid(2).MoI = [0.0 0.0 0.0];
smiData.Solid(2).PoI = [0.0 0.0 0.0];
smiData.Solid(2).color = [0.0 0.0 0.0];
smiData.Solid(2).opacity = 0.0;
smiData.Solid(2).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 63.712904832000014;  % kg
smiData.Solid(1).CoM = [254.00000000000006 95.25 -304.80000000000001];  % mm
smiData.Solid(1).MoI = [2227378.1456115735 3781661.6266900063 2062958.0748859206];  % kg*mm^2
smiData.Solid(1).PoI = [0 -5.2514416426507893e-10 246630.10608847879];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'SimpleSteps*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 141.58423296000004;  % kg
smiData.Solid(2).CoM = [-6.8172356065893798e-13 12.699999999999999 0];  % mm
smiData.Solid(2).MoI = [39468429.014467977 149074197.45792499 109620992.52407974];  % kg*mm^2
smiData.Solid(2).PoI = [0 2.2453105543718255e-09 0];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'Base*:*Default';

% Simscape(TM) Multibody(TM) version: 7.1

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(27).translation = [0.0 0.0 0.0];
smiData.RigidTransform(27).angle = 0.0;
smiData.RigidTransform(27).axis = [0.0 0.0 0.0];
smiData.RigidTransform(27).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-7.2299999999999951 0 2.660000000000001];  % in
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = 'B[simple_body-1:-:simple_intake_body-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [3.8775000000000013 2.6228346456692804 1.2653543300000001];  % in
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962584 0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(2).ID = 'F[simple_body-1:-:simple_intake_body-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0.25000000000000089 2.4228346456692882 0.20000000000000004];  % in
smiData.RigidTransform(3).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(3).axis = [1 0 0];
smiData.RigidTransform(3).ID = 'B[simple_intake_body-1:-:simple_intake_arm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [0.24999999999999967 -0.20000000000000018 0.24999999999999939];  % in
smiData.RigidTransform(4).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(4).axis = [-0.57735026918962584 -0.57735026918962573 -0.57735026918962573];
smiData.RigidTransform(4).ID = 'F[simple_intake_body-1:-:simple_intake_arm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [3.1250000000000013 2.4228346456692798 0.20000000000000004];  % in
smiData.RigidTransform(5).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(5).axis = [1 0 0];
smiData.RigidTransform(5).ID = 'B[simple_intake_body-1:-:simple_intake_arm-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [0.24999999999999756 -0.20000000000000018 0.25000000000000094];  % in
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(6).ID = 'F[simple_intake_body-1:-:simple_intake_arm-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-8.9199999999999964 0.64535432999999998 9.9500000000000011];  % in
smiData.RigidTransform(7).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(7).axis = [1 0 0];
smiData.RigidTransform(7).ID = 'B[simple_body-1:-:simple_leg-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0.24999999999999828 -0.25000000000000155 -0.24999999999999867];  % in
smiData.RigidTransform(8).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(8).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(8).ID = 'F[simple_body-1:-:simple_leg-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-0.48999999999999505 0.64535432999999998 9.9500000000000011];  % in
smiData.RigidTransform(9).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(9).axis = [1 0 0];
smiData.RigidTransform(9).ID = 'B[simple_body-1:-:simple_leg-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [0.25000000000000006 -0.25 -0.24999999999999994];  % in
smiData.RigidTransform(10).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(10).axis = [-0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(10).ID = 'F[simple_body-1:-:simple_leg-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [-0.48999999999999505 0.64535432999999998 -0.24999999999999981];  % in
smiData.RigidTransform(11).angle = 0;  % rad
smiData.RigidTransform(11).axis = [0 0 0];
smiData.RigidTransform(11).ID = 'B[simple_body-1:-:simple_leg-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [0.25000000000000011 -0.24999999999999922 -0.25000000000000033];  % in
smiData.RigidTransform(12).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(12).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(12).ID = 'F[simple_body-1:-:simple_leg-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-8.9199999999999964 0.64535432999999998 -0.24999999999999981];  % in
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = 'B[simple_body-1:-:simple_leg-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [0.250000000000001 -0.25000000000000339 -0.25000000000000133];  % in
smiData.RigidTransform(14).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(14).axis = [-0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(14).ID = 'F[simple_body-1:-:simple_leg-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [1.0600000000000003 -4.9400000000000013 -0.2499999999999987];  % in
smiData.RigidTransform(15).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(15).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(15).ID = 'B[simple_leg-3:-:simple_wheel-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [0 -1.2957581959731862e-15 -1.3857086914785453e-15];  % in
smiData.RigidTransform(16).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(16).axis = [0.57735026918962584 0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(16).ID = 'F[simple_leg-3:-:simple_wheel-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [1.0600000000000003 -4.9400000000000022 -0.25000000000000011];  % in
smiData.RigidTransform(17).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(17).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(17).ID = 'B[simple_leg-5:-:simple_wheel-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [0 -2.1246891101574191e-16 -1.8001370633861214e-15];  % in
smiData.RigidTransform(18).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(18).axis = [0.57735026918962573 0.57735026918962573 0.57735026918962562];
smiData.RigidTransform(18).ID = 'F[simple_leg-5:-:simple_wheel-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [1.0600000000000003 -4.9400000000000022 -0.25000000000000033];  % in
smiData.RigidTransform(19).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(19).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(19).ID = 'B[simple_leg-1:-:simple_wheel-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [0.39000000000000068 9.9920072216264089e-16 -2.7755575615628914e-15];  % in
smiData.RigidTransform(20).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(20).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(20).ID = 'F[simple_leg-1:-:simple_wheel-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [1.0600000000000003 -4.9400000000000004 -0.2499999999999987];  % in
smiData.RigidTransform(21).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(21).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(21).ID = 'B[simple_leg-2:-:simple_wheel-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [0.39000000000000246 8.8817841970012523e-16 1.5543122344752192e-15];  % in
smiData.RigidTransform(22).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(22).axis = [-0.57735026918962595 -0.57735026918962573 0.57735026918962562];
smiData.RigidTransform(22).ID = 'F[simple_leg-2:-:simple_wheel-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [0.24999999999999981 -1.0153543300000007 2.6742806400000001];  % in
smiData.RigidTransform(23).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(23).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(23).ID = 'B[simple_intake_arm-2:-:simple_wheel-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [0.39000000000000068 3.8857805861880479e-16 4.5380366131553274e-15];  % in
smiData.RigidTransform(24).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(24).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(24).ID = 'F[simple_intake_arm-2:-:simple_wheel-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [0.25000000000000089 -1.0153543300000007 2.674280640000001];  % in
smiData.RigidTransform(25).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(25).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(25).ID = 'B[simple_intake_arm-1:-:simple_wheel-6]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [0.39000000000000068 -1.4988010832439613e-15 3.2335245592207684e-15];  % in
smiData.RigidTransform(26).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(26).axis = [-0.57735026918962595 -0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(26).ID = 'F[simple_intake_arm-1:-:simple_wheel-6]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [9.0901746209389493 5.2926293822975206 3.0193211955852921];  % in
smiData.RigidTransform(27).angle = 0;  % rad
smiData.RigidTransform(27).axis = [0 0 0];
smiData.RigidTransform(27).ID = 'RootGround[simple_body-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(5).mass = 0.0;
smiData.Solid(5).CoM = [0.0 0.0 0.0];
smiData.Solid(5).MoI = [0.0 0.0 0.0];
smiData.Solid(5).PoI = [0.0 0.0 0.0];
smiData.Solid(5).color = [0.0 0.0 0.0];
smiData.Solid(5).opacity = 0.0;
smiData.Solid(5).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.013300000000000001;  % kg
smiData.Solid(1).CoM = [0.19499999999999998 0 0];  % in
smiData.Solid(1).MoI = [0.0038597712195424395 0.0021028024189335059 0.0021030379927473187];  % kg*in^2
smiData.Solid(1).PoI = [-7.5031246952745193e-07 0 0];  % kg*in^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'simple_wheel*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.008132207341052626;  % kg
smiData.Solid(2).CoM = [0.25 0.084224744436141474 1.5775449062228317];  % in
smiData.Solid(2).MoI = [0.006787179292400543 0.0064950076135401186 0.00060168003001639502];  % kg*in^2
smiData.Solid(2).PoI = [0.00057650014110219555 0 0];  % kg*in^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'simple_intake_arm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 1.7892999999999999;  % kg
smiData.Solid(3).CoM = [-5.4110236220472432 1.4846456685826774 4.3425196850393712];  % in
smiData.Solid(3).MoI = [21.618058574719416 18.193930284049433 35.902957356123579];  % kg*in^2
smiData.Solid(3).PoI = [1.8278926995325433 -0.63065966404074036 2.9989464940304811];  % kg*in^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'simple_body*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.051970583852045935;  % kg
smiData.Solid(4).CoM = [1.6874998531926935 1.9044020185767141 0.64487339891285933];  % in
smiData.Solid(4).MoI = [0.022007177353250699 0.02526519462352406 0.031461826708840401];  % kg*in^2
smiData.Solid(4).PoI = [0.0021728927074770557 4.4351125018240647e-09 -1.1134747985189163e-08];  % kg*in^2
smiData.Solid(4).color = [1 1 1];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'simple_intake_body*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.028879998475929206;  % kg
smiData.Solid(5).CoM = [0.15616237968352831 -2.7754340255749965 -0.24999999999999997];  % in
smiData.Solid(5).MoI = [0.068292852325626499 0.0012651472846890801 0.068394409926204561];  % kg*in^2
smiData.Solid(5).PoI = [0 0 0.0019480433095946774];  % kg*in^2
smiData.Solid(5).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'simple_leg*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(12).Rz.Pos = 0.0;
smiData.RevoluteJoint(12).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 87.913709619379063;  % deg
smiData.RevoluteJoint(1).ID = '[simple_intake_body-1:-:simple_intake_arm-1]';

smiData.RevoluteJoint(2).Rz.Pos = 92.483729353591926;  % deg
smiData.RevoluteJoint(2).ID = '[simple_intake_body-1:-:simple_intake_arm-2]';

smiData.RevoluteJoint(3).Rz.Pos = -100.4408328048652;  % deg
smiData.RevoluteJoint(3).ID = '[simple_body-1:-:simple_leg-1]';

smiData.RevoluteJoint(4).Rz.Pos = -90;  % deg
smiData.RevoluteJoint(4).ID = '[simple_body-1:-:simple_leg-2]';

smiData.RevoluteJoint(5).Rz.Pos = 89.568776082330785;  % deg
smiData.RevoluteJoint(5).ID = '[simple_body-1:-:simple_leg-3]';

smiData.RevoluteJoint(6).Rz.Pos = 90.112796546170912;  % deg
smiData.RevoluteJoint(6).ID = '[simple_body-1:-:simple_leg-5]';

smiData.RevoluteJoint(7).Rz.Pos = 22.953897503604001;  % deg
smiData.RevoluteJoint(7).ID = '[simple_leg-3:-:simple_wheel-1]';

smiData.RevoluteJoint(8).Rz.Pos = 108.1006322728355;  % deg
smiData.RevoluteJoint(8).ID = '[simple_leg-5:-:simple_wheel-2]';

smiData.RevoluteJoint(9).Rz.Pos = -77.997171507967295;  % deg
smiData.RevoluteJoint(9).ID = '[simple_leg-1:-:simple_wheel-3]';

smiData.RevoluteJoint(10).Rz.Pos = -22.522673585934783;  % deg
smiData.RevoluteJoint(10).ID = '[simple_leg-2:-:simple_wheel-4]';

smiData.RevoluteJoint(11).Rz.Pos = -148.16402258045068;  % deg
smiData.RevoluteJoint(11).ID = '[simple_intake_arm-2:-:simple_wheel-5]';

smiData.RevoluteJoint(12).Rz.Pos = -143.59400284623783;  % deg
smiData.RevoluteJoint(12).ID = '[simple_intake_arm-1:-:simple_wheel-6]';



