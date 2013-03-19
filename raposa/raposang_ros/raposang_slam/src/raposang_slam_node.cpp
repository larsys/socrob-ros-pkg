#include <raposang_slam.h>

#define VALUE_PI 3.141592654

static int cmp_func( const void* _a, const void* _b, void* userdata ){
    int* a = (int*)_a;
    int* b = (int*)_b;
    return (*a-*b); 
}

// ------------------------------------------------------------
//                      CLASS FUNCTIONS
// ------------------------------------------------------------

Slam::Slam(ros::NodeHandle nn){	
	
	double publish_per_second;
	this->n = nn;

	string feat_detector_name;

	it = new image_transport::ImageTransport (nn);
	
	ros::NodeHandle nh("~");
	
	nh.param("reads_per_second", params.reads_per_second, READS_PER_SECOND);
		
	nh.param("max_features", params.max_features, MAX_NEW_LANDMARKS);	
	nh.param("min_features_per_frame", params.min_features_per_frame, MIN_FEATURES_PER_FRAME);
	nh.param("min_dist", params.min_dist, MIN_DIST);	

	nh.param("center_to_lens", params.center_to_lens, 0.05);
	nh.param("use_gyro_only", params.use_gyro_only, true);
	
	nh.param("use_imu", params.use_imu, USE_IMU);	
	nh.param("use_odo", params.use_odo, USE_ODO);	
	nh.param("use_feat", params.use_feat, USE_FEAT);	
	
	nh.param("with_stereo", params.with_stereo, WITH_STEREO);
	nh.param("with_mono", params.with_mono, WITH_MONO);	

	nh.param("random_feats", params.random_feats, RANDOM_FEATS);
	
	nh.param("scale_stereo_str", params.scale_stereo_str, SCALE_STEREO_STR);
	nh.param("arm_angle_always_zero", params.arm_angle_always_zero, ARM_ANGLE_ALWAYS_ZERO);	
	
	nh.param("calibrate_imu", params.calibrate_imu, CALIBRATE_IMU);

	nh.param("feature_removal", params.feature_removal, FEATURE_REMOVAL);	
	
	sz = 7;
	if (!params.use_imu)
		sz += 3;
	if (!params.use_odo)
		sz += 3;		

	sz_w = (params.use_odo) ? 7 : 10;	

	nh.param("do_update", params.do_update, true);	

	//nh.param("wheel_diameter", params.wheel_diameter, WHEEL_DIAMETER);	
	nh.param("wheel_distance", params.wheel_distance, WHEEL_DISTANCE);	
		
	nh.param<std::string>("feature_detector", feat_detector_name, FEATURE_DETECTOR);
	
	if (feat_detector_name == "surf") {
		params.feature_detector = fd_surf;
		
		nh.param("surf_extended_descriptors", SURFparams.extended, 					(int) SURF_EXTENDED_DESCRIPTORS);		
		nh.param("surf_hessian_threshold", 	  SURFparams.hessianThreshold, 	SURF_HESSIAN_THRESHOLD);
		nh.param("surf_nr_octaves",					  SURFparams.nOctaves,  				SURF_NR_OCTAVES);	
		nh.param("surf_nr_octave_layers", 	  SURFparams.nOctaveLayers, 		SURF_NR_OCTAVE_LAYERS);

		params.descr_size = (SURFparams.extended ? 128 : 64)*sizeof(float);	
			
	}
	else if (feat_detector_name == "orb") {
		params.feature_detector = fd_orb;	
		params.descr_size = 32;			
	}

	nh.param<std::string>("frameid_map", params.frameid_map, FRAMEID_MAP);
	nh.param<std::string>("frameid_robot", params.frameid_robot, FRAMEID_ROBOT);
	
	nh.param("focal_lenght_x", params.focal_lenght_x, IMAGE_FOCAL_X);
	nh.param("focal_lenght_y", params.focal_lenght_y, IMAGE_FOCAL_Y);

	nh.param("image_size_x", params.image_size_x, IMAGE_SIZE_X);
	nh.param("image_size_y", params.image_size_y, IMAGE_SIZE_Y);

	nh.param("image_scale", params.image_scale, IMAGE_SCALE);

	nh.param("image_center_x", params.image_center_x, IMAGE_ORIGIN_X);
	nh.param("image_center_y", params.image_center_y, IMAGE_ORIGIN_Y);

	nh.param("removal_gain", 			params.removal_gain, REMOVAL_GAIN);
	nh.param("removal_threshold", params.removal_threshold, REMOVAL_THRESHOLD);
	
	nh.param("init_std_lin_vel", params.init_std_lin_vel, INIT_STD_LIN_VEL);
	nh.param("init_std_ang_vel", params.init_std_ang_vel, INIT_STD_ANG_VEL);
	nh.param("init_std_inv_depth",   params.init_std_inv_depth, INIT_STD_INV_DEPTH);
	
	nh.param("init_inv_depth", 	 params.init_inv_depth,	  INIT_INV_DEPTH);	

	nh.param("std_lin_acc", 		params.std_lin_acc, 	STD_LIN_VEL);
	nh.param("std_ang_acc",			params.std_ang_acc, 	STD_ANG_VEL);	
	nh.param("std_imu_vel",  		params.std_imu_vel, 	STD_IMU_VEL);
	nh.param("std_wheel_vel",		params.std_wheel_vel, STD_WHEEL_VEL);		
	nh.param("std_arm",					params.std_arm, 			STD_ARM);	
	
	nh.param("std_pixel", 		params.std_pixel, 		STD_PIXEL_IMG);

	params.focal_lenght_x *= params.image_scale;
	params.focal_lenght_y *= params.image_scale;		
	params.image_size_x   *= params.image_scale;
	params.image_size_y   *= params.image_scale;
	params.image_center_x *= params.image_scale;
	params.image_center_y *= params.image_scale;	
					
	imleft.create(params.image_size_y, params.image_size_x, CV_8UC1);	
	imleft_graph.create(params.image_size_y, params.image_size_x, CV_8UC3);	

	sub_Odo = n.subscribe("input_odo", 1, &Slam::callbackOdo, this);

	if (params.use_imu) 
		sub_IMU = n.subscribe("input_imu", 1, &Slam::callbackIMU, this);	

	if (!params.use_feat)
		sub_Img_L = it->subscribe("input_image", 1, &Slam::callbackImage, this);
	else
		sub_Feat = n.subscribe("input_feat", 1, &Slam::callbackFeatures, this);

	pubIL = it->advertise("image_data", 1);	
	pub = n.advertise<raposang_msgs::RaposaSlam>("pose", 1);	
	pub_data = n.advertise<sensor_msgs::PointCloud>("landmarks", 1);	
		
	nh.param("publish_per_second", publish_per_second, 1.0);
	sub_Pub = n.createTimer(ros::Duration(1.0/publish_per_second), &Slam::callbackPublish, this);
	
}

Slam::~Slam(){
}

// ------------------------------------------------------------
//                     AUX FUNCTIONS
// ------------------------------------------------------------

Matrix<double,3,4> Slam::quaternion_dRqM_by_dq(double w, double x, double y, double z, Vector3d M){
			
	w *= 2;	x *= 2;	y *= 2;	z *= 2;
	
	Matrix3d M0, M1, M2, M3;
	Matrix<double,3,4> u;

	M0 <<  w, -z,  y,
	       z,  w, -x,
	      -y,  x,  w;
	      
	M1 <<  x,  y,  z,
	       y, -x, -w,
	       z,  w, -x;	 
	            
	M2 << -y,  x,  w,
	       x,  y,  z,
	      -w,  z, -y;
	      
	M3 << -z, -w,  x,
	       w, -z,  y,
	       x,  y,  z;
		
	u.col(0) = M0*M;
	u.col(1) = M1*M;
	u.col(2) = M2*M;	     
	u.col(3) = M3*M;	 
	
	return u;
}

void Slam::ekf(const double dt) {

	vector<int> pairs;
	
	feat_removed = 0;
	feat_added = 0;	
	
	// -------------------------------
	// Extended Kalman Filter: Predict
	// -------------------------------
	
	Slam::predict(dt);	

	if (got_image && !first_image && params.do_update) {

		ROS_INFO("Ekf: Starting new iteration, time of last call = %f", dt);

		// ------------------------------------------------
		// Matching between landmarks in state and features
		// ------------------------------------------------

		pairs = Slam::matching();

		// ------------------------------
		// Extended Kalman Filter: Update
		// ------------------------------
		
		if (im.matched)
			Slam::update(pairs);
		else 
			visible_and_detected = 0;
			
		// ------------------------
		// Remove unwanted features
		// ------------------------		

		if (params.feature_removal)
			Slam::removeFeatures();

		// -------------------	
		// Insert new features
		// -------------------
		
		if ((params.max_features - s.l.total) > 0)
			Slam::insertFeatures();

	}
	
}

// ------------------------------------------------------------
//                        PREDICT
// ------------------------------------------------------------

void Slam::predict(const double dt) {

	//ROS_INFO("Predict: Starting...");

	const int total = sz + 6*s.l.total;	

	double T = 0.0, ang = 0.0;
	Matrix3d Rq;
	Vector3d pos, pos_odo, zarm;	

	const double dth = dt/2.0;	
		
	double wx, wy, wz;

	if (params.use_imu) {		
		wx = w_imu.x;
		wy = w_imu.y;
		wz = w_imu.z;		
	} else {

			wx = s.x(sz_w);
			wy = s.x(sz_w+1);
			wz = s.x(sz_w+2);

	}
	
	const double wn  = sqrt(wx*wx + wy*wy + wz*wz);
	const double swn = sin(dth*wn);
	const double cwn = cos(dth*wn);	
	const double wxn = (wn != 0) ? wx/wn : 0.0;	// Normalized angular velocity for x
	const double wyn = (wn != 0) ? wy/wn : 0.0;	// Normalized angular velocity for y
	const double wzn = (wn != 0) ? wz/wn : 0.0;	// Normalized angular velocity for z
	
	const double swnn = (wn != 0) ? swn/wn : dth;
	const double a    = dth*cwn - swnn;		

	Quaterniond	q(s.x(3),s.x(4),s.x(5),s.x(6));	
	Quaterniond	q_odo(s.x_odo(3),s.x_odo(4),s.x_odo(5),s.x_odo(6));		
	Quaterniond qR(cwn, swn*wxn, swn*wyn, swn*wzn);
	Quaterniond	qnew, qnew_odo;		

	// --------------------
	// Predict new position
	// --------------------
	
	Matrix3d Rarm, Rq_odo;
	Vector3d todo;
	Vector3d tarm(0.0, -0.17, -0.21);
	Vector3d timu;

	if (params.use_odo) {
		
		T = (odo_dif.left + odo_dif.right)/2.0;
		ang = (odo_dif.right - odo_dif.left)/(params.wheel_distance);
		
		Rq = q.toRotationMatrix();		
				
		Rq_odo = q_odo.toRotationMatrix();	
		
		todo << T*sin(ang), 0, T*cos(ang);	
		
		Rarm << 1, 				 		0, 				    	0,
				    0,  cos(odo_arm),  sin(odo_arm),
				    0, -sin(odo_arm),  cos(odo_arm);				
		
		timu = tarm + Rarm * todo - qR.toRotationMatrix() * tarm;
		
		pos = s.x.head<3>() + Rq * timu;
		pos_odo = s.x_odo.head<3>() + Rq_odo * timu;

		
	} else 	{
	  pos = s.x.head<3>() + s.x.segment<3>(7) * dt;
	  pos_odo = s.x_odo.head<3>();	
	}
	// -----------------------
	// Predict new orientation
	// -----------------------
	
	if (wn) {
		qnew = q * qR;
		qnew_odo = q_odo * qR;
		qnew_odo.normalize();
	}
	else {
		qnew = q;
		qnew_odo = q_odo;		
	}
	
	// -------------------------------
	// Calculate Transition Matrix "F"
	// -------------------------------
	
	MatrixXd F(sz,sz);	
	F.setIdentity();
		
	if (params.use_odo)		
		F.block<3,4>(0,3) = quaternion_dRqM_by_dq(s.x(3),s.x(4),s.x(5),s.x(6),timu);
	else 
		F(0,7) = F(1,8) = F(2,9) = dt;

	F.block<4,4>(3,3) << qR.w(), -qR.x(), -qR.y(), -qR.z(),
											 qR.x(),  qR.w(),  qR.z(), -qR.y(),
											 qR.y(), -qR.z(),  qR.w(),  qR.x(),
											 qR.z(),  qR.y(), -qR.x(),  qR.w();
								 			 
 	Matrix4d dqnew_by_dqR;
 	Matrix<double,4,3> dqR_by_dw;
 	
	dqnew_by_dqR <<  q.w(), -q.x(), -q.y(), -q.z(),
									 q.x(),  q.w(), -q.z(),  q.y(),
									 q.y(),  q.z(),  q.w(), -q.x(),
									 q.z(), -q.y(),  q.x(),  q.w();
		
	dqR_by_dw << -dth*wxn*swn,   -dth*wyn*swn,   -dth*wzn*swn, 
								a*wxn*wxn+swnn, a*wxn*wyn,      a*wxn*wzn,
								a*wyn*wxn,      a*wyn*wyn+swnn, a*wyn*wzn,
								a*wzn*wxn,      a*wzn*wyn,      a*wzn*wzn+swnn;

	if (!params.use_imu)
		F.block<4,3>(3,sz_w) = dqnew_by_dqR * dqR_by_dw;	
	
	// -----------------------------------------
	// Calculate Transition Error Covariance "Q"
	// -----------------------------------------
 
	Matrix6d Pn;	
 	MatrixXd G(sz,6);	
 	MatrixXd Q(sz,sz);

 	Pn.setZero();
 	G.setZero(); 
 	 	
	if (params.use_odo)	{	
		Pn(0,0) = Pn(1,1) = params.std_wheel_vel*params.std_wheel_vel*dt*dt;
		Pn(2,2) = params.std_arm*params.std_arm;
		// rL

		G.block<3,1>(0,0) = Rq * Rarm * Vector3d(0.5 * sin(ang) - T * cos(ang) / params.wheel_distance, 
																						 0.0,
																						 0.5 * cos(ang) + T * sin(ang) / params.wheel_distance);	
																						 	
		G.block<3,1>(0,1) = Rq * Rarm * Vector3d(0.5 * sin(ang) + T * cos(ang) / params.wheel_distance, 
																						 0.0,
																						 0.5 * cos(ang) - T * sin(ang) / params.wheel_distance);	
																						 
		G.block<3,1>(0,2) = Rq * T * cos(ang) * Vector3d(0.0, cos(odo_arm), -sin(odo_arm));	

		G.block<3,3>(0,3) = - Rq * quaternion_dRqM_by_dq(qR.w(), qR.x(), qR.y(), qR.z(), tarm) * dqR_by_dw;
		
		//cout << G.block<3,3>(0,3)<< "\n\n";	
	}
	else {
		Pn(0,0) = Pn(1,1) = Pn(2,2) = params.std_lin_acc*params.std_lin_acc*dt*dt;
		
		G(0,0) = G(1,1) = G(2,2) = dt;	
		G.block<3,3>(7,0).setIdentity();
	}
	
	if (params.use_imu)	{
		Pn(3,3) = Pn(4,4) = Pn(5,5) = params.std_imu_vel*params.std_imu_vel;
			
		G.block<4,3>(3,3) = dqnew_by_dqR * dqR_by_dw;	
	}
	else {
		Pn(3,3) = Pn(4,4) = Pn(5,5) = params.std_ang_acc*params.std_ang_acc*dt*dt;
		
		G.block<4,3>(3,3) = F.block<4,3>(3,sz_w);	
		G.block<3,3>(sz_w,3).setIdentity();
	}

	Q = G*Pn*G.transpose();

	// ------------
	// Predict Step
	// ------------

	s.x.head<7>() << pos, qnew.w(), qnew.x(), qnew.y(), qnew.z();
	s.x_odo << pos_odo, qnew_odo.w(), qnew_odo.x(), qnew_odo.y(), qnew_odo.z();
	
	s.P.block(sz,0,total-sz,sz) = s.P.block(sz,0,total-sz,sz).eval() * F.transpose();
	s.P.block(0,sz,sz,total-sz) = s.P.block(sz,0,total-sz,sz).transpose();		
		
	s.P.topLeftCorner(sz,sz) = F * s.P.topLeftCorner(sz,sz).eval() * F.transpose() + Q;		

	if (params.use_imu) 
		update_imu = true;

	//if (params.use_odo) 
	
	update_odo = true;
				
	//ROS_INFO("Predict: Done!");

}

// ------------------------------------------------------------
//                       MATCHING
// ------------------------------------------------------------

vector<int> Slam::matching() {

	//ROS_INFO("Match: Starting...");
	
	int i;
	
	vector<int> pairs;	
	vector<vector<cv::DMatch> > matches;
	
	if (im.total>0 && s.l.total>0) {
	
		if(params.feature_detector==fd_surf){		
			BruteForceMatcher<L2<float> > matcher_l;
			matcher_l.knnMatch(s.l.d, im.d, matches, 2);
		} else if(params.feature_detector==fd_orb){		
			BruteForceMatcher<Hamming> matcher_h;
			matcher_h.knnMatch(s.l.d, im.d, matches, 2);
		} 
	
		float ratio = 0.7f;
		for(i=0; i < matches.size(); i++) {	
			////ROS_INFO("%f %f", matches[i][0].distance, matches[i][1].distance);
			////ROS_INFO("%f %f", matches[i][0].distance, matches[i][1].distance);
			if(matches[i][0].distance < matches[i][1].distance*ratio) {
					pairs.push_back(matches[i][0].queryIdx);
					pairs.push_back(matches[i][0].trainIdx);		
				}
		}

		im.matched = pairs.size()/2;
	}
	else
		im.matched = 0;
	
	//ROS_INFO("Match: Done! %d matches between %d landmarks and %d features.", im.matched, s.l.total, im.total);
		
	return pairs;
		
}

// ------------------------------------------------------------
//                      UPDATE IMAGE
// ------------------------------------------------------------

void Slam::update(vector<int>& pairs) {

	//ROS_INFO("Update: Starting...");
			
	int i, j, ii, k;
	double zx, hx_cam, hx, hy, cya, cye, sya, sye, dif_zx_hx, dif_zy_hy;
	bool visible, detected;
		
	const double var_pixel = params.std_pixel*params.std_pixel;
	
	const int nh1   = pairs.size()/2;
	const int nh2   = 2*nh1;
	const int nh3   = 3*nh1;
	const int total = sz + 6*s.l.total;		
	const float min_dist_sqr = params.min_dist*params.min_dist;
	
	cv::KeyPoint z;
	
	VectorXd z_h;
	z_h.setZero(nh3);
	
	MatrixXd H;
	H.setZero(nh3, total);	
	
	// --------------------------------------------------------------
	// Calculate Rotation Matrices from robot to world and vice-versa
	// --------------------------------------------------------------
	
	Matrix3d Rc, iRc;
	Quaterniond	q(s.x(3),s.x(4),s.x(5),s.x(6));		
	Rc  = q.toRotationMatrix();
	iRc = Rc.transpose();

	// ------------------------------------
	// Calculate dq/dq* for posterior usage
	// ------------------------------------
	
	Matrix4d DiagSp;	
	DiagSp.setZero();
	DiagSp(0,0) = 1.0;
	DiagSp(1,1) = DiagSp(2,2) = DiagSp(3,3) = -1.0; 

		
	Vector3d lenc, lenw;
	lenc << params.center_to_lens, 0, 0;
	lenw = Rc * lenc;		
	
	// ---------------------------------------------------------
	// Check if each landmark in state is visible and detectable
	// ---------------------------------------------------------

	int so_visible = 0;
	visible_and_detected = 0;

	
	for(i=0, k=0; i<s.l.total; i++) {
	
		// -----------------
		// Observation Model
		// -----------------
		
		Vector6d yi;	
		Vector3d m, hc, hw, rwc;		
		
		yi = s.x.segment<6>(sz+6*i);		

		cya = cos(yi(3));	cye = cos(yi(4));
		sya = sin(yi(3));	sye = sin(yi(4));
		m << cye*sya, -sye, cye*cya;		
		
		rwc = yi.head<3>() - s.x.head<3>();
		hw  = yi(5) * rwc + m;
		hc  = iRc * hw;
		
		hx_cam =  params.focal_lenght_x * params.center_to_lens * yi(5) / hc(2); 
		hx 		 =  params.image_center_x - ( hc(0)/hc(2) ) * params.focal_lenght_x;
		hy 		 =  params.image_center_y - ( hc(1)/hc(2) ) * params.focal_lenght_y;
	
		// -----------------------------------------------------
		// Check if it should be visible on screen by any camera
		// -----------------------------------------------------

		visible = (yi(5) * hc(2) >= 0.0) && 
//							hx>-hx_cam && hx<params.image_size_x+hx_cam &&
							hx>0 && hx<params.image_size_x &&
							hy>0 && hy<params.image_size_y;
				
		// -----------
		// If visible:
		// -----------

		if(visible) {
	
			so_visible++;
	
			// ----------------------------------------------------
			// Check if it matches with any features currently seen
			// ----------------------------------------------------
			
			detected = false;
			for(j = 0; j < nh2; j+=2) {
				if(pairs[j]==i) {	
					
					ii = pairs[j+1];
					
					z = im.k[ii];	

					zx = (im.id_type(ii) == cam_stereo) ? ((z.pt.x + z.angle)/2.0) : z.pt.x;

					dif_zx_hx = zx - (hx + im.id_type(ii)*hx_cam);		
					dif_zy_hy = z.pt.y - hy;

					if(dif_zx_hx*dif_zx_hx + dif_zy_hy*dif_zy_hy < min_dist_sqr) {
						
						im.id_real(ii) = 1;
			
						s.l.k[i] = z;
									
						s.l.d.row(i) = im.d.row(ii) + 0.0;
						
						detected = true;						
						break;						
					}
				}			
			}
			
			// ------------
			// If detected:
			// ------------
			
			if(detected) {

				visible_and_detected++;

				// ------------------------------------------
				// Draw Plot (white: real / black: predicted)
				// ------------------------------------------
	
				cv::line(imleft_graph, cv::Point(zx, z.pt.y), cv::Point(hx+im.id_type(ii)*hx_cam, hy), CV_RGB(0,0,255), 4);
				cv::circle(imleft_graph, cv::Point(zx, z.pt.y), 4, CV_RGB(0,255,0), -1);	

				// ------------------------------------------------
				// Calculate Observation Matrix "H" for feature "i"
				// ------------------------------------------------				

				Matrix<double,2,3> dz_dhc;
				Matrix<double,3,3> dz_dhc_stereo;
				Matrix<double,3,6> dhw_dyi;

				dhw_dyi.topLeftCorner<3,3>().setIdentity();
				dhw_dyi(0,0) = dhw_dyi(1,1) = dhw_dyi(2,2) = yi(5);
				dhw_dyi.col(3) <<  cye*cya,    0, -cye*sya;
				dhw_dyi.col(4) << -sye*sya, -cye, -sye*cya;																	 
				dhw_dyi.col(5) = rwc;
				
				if(im.id_type(ii) != cam_stereo) {
					
					z_h(k)   = z.pt.x  - (hx + im.id_type(ii)*hx_cam);			
					z_h(k+1) = z.pt.y  -  hy;		

					dz_dhc << -params.focal_lenght_x/hc(2),  0, (hc(0)-im.id_type(ii)*yi(5)*params.center_to_lens)*params.focal_lenght_x/(hc(2)*hc(2)),
										0, -params.focal_lenght_y/hc(2), hc(1)*params.focal_lenght_y/(hc(2)*hc(2));				
										
					H.block<2,3>(k,0).noalias() = dz_dhc * iRc * -yi(5);
					
					H.block<2,4>(k,3).noalias() = dz_dhc
										* Slam::quaternion_dRqM_by_dq(s.x(3),-s.x(4),-s.x(5),-s.x(6), hw) 
										* DiagSp;
					
					H.block<2,6>(k,sz+6*i).noalias() = dz_dhc * iRc * dhw_dyi;
					H(k,sz+6*i+5) += im.id_type(ii) * params.center_to_lens * params.focal_lenght_x / hc(2);
					
					k += 2;
					
				} 
				else {

					z_h(k)   = z.pt.x  - (hx + hx_cam);
					z_h(k+1) = z.angle - (hx - hx_cam);					
					z_h(k+2) = z.pt.y  -  hy;					

					dz_dhc_stereo << -params.focal_lenght_x/hc(2),  0, (hc(0)-yi(5)*params.center_to_lens)*params.focal_lenght_x/(hc(2)*hc(2)),
													 -params.focal_lenght_x/hc(2),  0, (hc(0)+yi(5)*params.center_to_lens)*params.focal_lenght_x/(hc(2)*hc(2)),
													  0, -params.focal_lenght_y/hc(2), hc(1)*params.focal_lenght_y/(hc(2)*hc(2));	
					
					H.block<3,3>(k,0).noalias() = dz_dhc_stereo * iRc * -yi(5);
					
					H.block<3,4>(k,3).noalias() = dz_dhc_stereo
										* Slam::quaternion_dRqM_by_dq(s.x(3),-s.x(4),-s.x(5),-s.x(6), hw) 
										* DiagSp;
					
					H.block<3,6>(k,sz+6*i).noalias() = dz_dhc_stereo * iRc * dhw_dyi;
					H(k  ,sz+6*i+5) += params.center_to_lens * params.focal_lenght_x / hc(2);
					H(k+1,sz+6*i+5) -= params.center_to_lens * params.focal_lenght_x / hc(2);
								
					k += 3;			
										
				}
			
			}  
			else {
			
				// -----------------------
				// Update Importance Value
				// -----------------------
						
				s.f(i) *= params.removal_gain;
				
			}				
		} else {
			
			// ---------------------------------------
			// Discard features with negative depth
			// ---------------------------------------	
			
			if (yi(5) < 0.0)  s.f(i) = 0;
			
		}			
	}

	z_h.conservativeResize(k);
	H.conservativeResize(k,total);

	if(k>0) {
		
		MatrixXd S(k,k), P_Ht, K, I_minus_KH(total,total);

		// ------------------------------
		// Calculate Innovance Matrix "S"
		// ------------------------------
	
		P_Ht.noalias() = s.P*H.transpose();	
		S.noalias() = H*P_Ht;
		for(i=0;i<k;i++)
			S(i,i)+=var_pixel;				

		// -------------------------
		// Calculate Kalman Gain "K"
		// -------------------------

		K.noalias() = P_Ht*S.inverse();

		// -----------
		// Update Step
		// -----------	
		
		I_minus_KH.noalias() = -K*H;
		for(i=0;i<total;i++)
			I_minus_KH(i,i)++;		
		
		s.x.noalias() += K*z_h;
		
		s.P = I_minus_KH*s.P;		

		// ---------------------
		// Normalize Orientation
		// ---------------------
		
		Slam::normalizeQuaternionInState();

		ROS_INFO("Update: Done! z=%f, %d of %d landmarks matched.", s.x(1), visible_and_detected, so_visible);

		
	}	
}

// ------------------------------------------------------------
//              NORMALIZE QUATERNION IN STATE
// ------------------------------------------------------------

void Slam::normalizeQuaternionInState() {		
	
	Matrix4d J;
	
	const int total = sz + 6*s.l.total;	
	
	const double qw = s.x(3), qy = s.x(4), qx = s.x(5), qz = s.x(6);
	const double qww = qw*qw, qxx = qx*qx, qyy = qy*qy, qzz = qz*qz;
	const double iqnorm = 1.0/sqrt(qww + qxx + qyy + qzz);
	
	J << qxx+qyy+qzz,      -qw*qx,      -qw*qy,      -qw*qz,
						-qx*qw, qww+qyy+qzz,      -qx*qy,      -qx*qz,
						-qy*qw,      -qy*qx, qww+qxx+qzz,      -qy*qz,
						-qz*qw,      -qz*qx,      -qz*qy, qww+qxx+qyy;
	
	J *= iqnorm*iqnorm*iqnorm;
	
	s.x.segment<4>(3) *= iqnorm;
	
	s.P.block(0,3,total,4) = s.P.block(0,3,total,4).eval()*J;
	s.P.block(3,0,4,total) = J*s.P.block(3,0,4,total).eval();	
	
}

// ------------------------------------------------------------
//                    INSERT FEATURES
// ------------------------------------------------------------

void Slam::insertFeatures() {	

	//ROS_INFO("Insert: Starting...");
	
	int i, k, j, i_rand;
	float str, aux_str; 
	int new_state_size, pos, pos_f;
	int newlnd[params.max_features]; 
	
	int add = im.total-im.id_real.sum();
	
	int nrfeats = std::min(add, std::min(params.max_features - s.l.total, params.min_features_per_frame));	

  srand(time(NULL));

	// --------------------------------------
	// Find good candidates for new landmarks	
	// --------------------------------------

	add = im.total-im.id_real.sum();

	for(k=0;k<nrfeats;) {
		
		newlnd[k] = -1;
		aux_str = -1.0;
		
		if (params.random_feats) {

			do {
				i_rand = rand() % im.total;
			} while(im.id_real(i_rand) == 1);

			newlnd[k] = i_rand;
			im.id_real(i_rand) = 1;
		
		} else {
			
			for(i=0;i<im.total;i++){

				str = im.k[i].response;			
				
				if((im.id_real(i)==0) && (str > aux_str)) {
					newlnd[k] = i;
					aux_str = str;
				}		
			} 
		}
		
		if(newlnd[k]!=-1) {
			im.id_real(newlnd[k]) = 1;
			k++;
		}
		else
			break;
	}

	// ----------------------------------
	// If new landmarks will be inserted:
	// ----------------------------------
	
	if(k>0) {	
		
		Vector3d h, n;		
		Matrix3d Rq, Paux;
		Matrix<double, 6, 7> J1;
		Matrix<double, 6, 3> J2;
		Matrix<double, 2, 3> dangles_dn;
		Matrix<double, 3, 4> dn_dq;
		Matrix<double, 3, 2> dh_duv;
		Matrix<double, 3, 3> dh_duv_stereo;
		Quaterniond	q(s.x(3),s.x(4),s.x(5),s.x(6));
					
		double nxz, nxzxz, naux;

		Rq = q.toRotationMatrix();

		Vector3d lenc, cam_pos;
		lenc << params.center_to_lens, 0, 0;
		
		J1.setZero();
		J1.topLeftCorner(3,3).setIdentity();
		
		J2.setZero();		
		
		pos = sz+6*s.l.total;
		pos_f = s.l.total;
		new_state_size = pos+6*k;				

		s.x.conservativeResize(new_state_size);
		s.P.conservativeResize(new_state_size,new_state_size);
		
		s.l.total+=k;					
		s.f.conservativeResize(s.l.total);			

		Paux.setZero();
		Paux(0,0) = Paux(1,1) = params.std_pixel*params.std_pixel;

		// ----------------------------------------------------
		// Insert selected candidates as new landmarks in state
		// ----------------------------------------------------

		if (!s.l.total)
			s.l.d.create(0,im.d.cols, im.d.type());	

		for(i=0;i<k;i++){	

			j = newlnd[i];

			//ROS_INFO("New landmark %d", j);

			cv::circle(imleft_graph, im.k[j].pt, 3, CV_RGB(255,255,0), -1);	
			
			s.l.k.push_back(im.k[j]);	
			s.l.d.push_back(im.d.row(j));	
													
			s.f(pos_f) = REMOVAL_START;

			cam_pos = s.x.head(3) + im.id_type(j) * Rq * lenc;

			double ptzx = (im.id_type(j) == cam_stereo) ? ((im.k[j].pt.x + im.k[j].angle)/2.0) : im.k[j].pt.x;

			h << (params.image_center_x - ptzx)         / params.focal_lenght_x,
					 (params.image_center_y - im.k[j].pt.y) / params.focal_lenght_y,
						1.0;	

			n = Rq*h;
			
			nxzxz = n(0)*n(0)+n(2)*n(2);
			nxz   = sqrt(nxzxz);
			naux  = 1.0/((n(1)*n(1)+nxzxz)*nxz);
			
			s.x.segment<5>(pos) << cam_pos(0), cam_pos(1), cam_pos(2), 
									 atan2( n(0), n(2)),
									 atan2(-n(1), nxz);	
									 	
			dangles_dn << n(2)/nxzxz, 0, -n(0)/nxzxz,
							      n(0)*n(1)*naux, -nxzxz*naux, n(1)*n(2)*naux;
								
			double bfm = 2.0 * params.center_to_lens * params.focal_lenght_x;					 
			double zlr = (im.k[j].pt.x - im.k[j].angle);
			double hnorm = h.norm();
			double bfm2hnorm3 = bfm * params.focal_lenght_x * hnorm*hnorm*hnorm;
	
			J2.setZero();		
									 
			if (im.id_type(j) == cam_stereo){
			
				Paux(2,2) = params.std_pixel*params.std_pixel;

				J1.block<3,4>(0,3).setZero();
				
				s.x(pos+5) = zlr/(bfm*hnorm);
				
				dh_duv_stereo << -1.0/(2.0*params.focal_lenght_x), -1.0/(2.0*params.focal_lenght_x), 0,
													0, 0, -1.0/params.focal_lenght_y,
													0, 0, 0;
				
				J2.block<2,3>(3,0).noalias() = dangles_dn * Rq * dh_duv_stereo;					
				J2.block<1,3>(5,0) <<  1.0/(bfm*hnorm) + zlr*h(0)/(2*bfm2hnorm3),
															-1.0/(bfm*hnorm) + zlr*h(0)/(2*bfm2hnorm3),					
																								 zlr*h(1)/(  bfm2hnorm3);
			}
			else {
				
				Paux(2,2) = params.init_std_inv_depth*params.init_std_inv_depth; 				

				J1.block<3,4>(0,3).noalias() = quaternion_dRqM_by_dq(s.x(3),s.x(4),s.x(5),s.x(6), lenc);			
			
				s.x(pos+5) = params.init_inv_depth;
				
				dh_duv << -1.0/params.focal_lenght_x, 0, 
									0, -1.0/params.focal_lenght_y,
									0, 0;	
															
				J2.block<2,2>(3,0).noalias() = dangles_dn * Rq * dh_duv;
				J2(5,2) = 1.0;
					
			}
				
			J1.block<2,4>(3,3).noalias() = dangles_dn * Slam::quaternion_dRqM_by_dq(s.x(3),s.x(4),s.x(5),s.x(6),h);					

			s.P.block(0,pos,pos,6)   = s.P.block(0,0,pos,7)*J1.transpose();			
			s.P.block(pos,0,6,pos+6) = J1*s.P.block(0,0,7,pos+6);			
			s.P.block<6,6>(pos,pos).noalias() += J2*Paux*J2.transpose();			
										 			 																		 			 
			pos += 6;
			pos_f++;
		}

	}

	//ROS_INFO("Insert: Done ! %d new features. Total is %d", k, s.l.total);
		
}

// ------------------------------------------------------------
//                    REMOVE FEATURES
// ------------------------------------------------------------

void Slam::removeFeatures() {	
	
	//ROS_INFO("Remove: Starting...");	
	
	int i, j, jk, k, r;
	CvSeqReader f_reader;

	const int extra = params.min_features_per_frame - visible_and_detected;

	const int total = sz+6*s.l.total;

	// --------------------------------------
	// Fill list with landmarks to be removed
	// --------------------------------------

	CvMemStorage *f_storage = cvCreateMemStorage(0); 	
	CvSeq *filter = cvCreateSeq(0, sizeof(CvSeq), sizeof(int), f_storage);

	for(k=0;(k<extra) && (k<s.l.total);k++) 
		cvSeqPush(filter, (int *) &k);

/*
	for(k=0;k<extra;) {
		
		aux = 2.0;
		ind = -1;
		
		for(i=0;i<s.l.total;i++){		
			if (s.f(i)<aux){
				aux = s.f(i);
				ind = i;
			}		
		}

		if(ind != -1) {
			cvSeqPush(filter, (int *) &ind);
			s.f(ind) = 2.0;	
			k++;
		}
		else
			break;
	} */

	for(;k<s.l.total;k++)
		if(s.f(k)<params.removal_threshold)	
			cvSeqPush(filter, (int *) &k);	

	if(extra>0)
		cvSeqSort(filter, cmp_func, 0);
		
	cvStartReadSeq(filter, &f_reader, 0);
	
	for(i=0;i<filter->total;i++) 
		CV_NEXT_SEQ_ELEM(f_reader.seq->elem_size, f_reader);

	feat_removed = filter->total;

	// -------------------------------------
	// If there are landmarks to be removed:
	// -------------------------------------

	if (filter->total > 0) {
	
		cvStartReadSeq(filter, &f_reader, 0);		
		r = *f_reader.ptr;	
		k = 0;
		
		for(i=r; i<s.l.total; i++) {

			if(r == i) {		
	
			//	cvSeqRemove(s.l.k, r - k);
			//	cvSeqRemove(s.l.d, r - k);	
				
				CV_NEXT_SEQ_ELEM(f_reader.seq->elem_size, f_reader);					
				r = *f_reader.ptr;										
				k++;
				
			} else {
		
				j  = sz+6*i;		
				jk = sz+6*(i-k);
				
				s.x.segment<6>(jk) = s.x.segment<6>(j);
				s.P.block(jk,0,6,total) = s.P.block(j,0,6,total);
				s.P.block(0,jk,total,6) = s.P.block(0,j,total,6);	
				
				s.l.k[i-k] = s.l.k[i];
				s.l.d.row(i-k) = s.l.d.row(i) + 0.0;
		
				s.f(i-k) = s.f(i);
			}
		}
	
		s.l.total -= filter->total;	
		
		j = sz+6*s.l.total;
		s.x.conservativeResize(j);
		s.P.conservativeResize(j,j);
		s.f.conservativeResize(s.l.total);
		
		s.l.k.resize(s.l.total);
		s.l.d.resize(s.l.total);

	}

	//ROS_INFO("Remove: Done! %d landmarks were removed.", filter->total);
		
	cvReleaseMemStorage(&f_storage); 
}

void Slam::writeLandmarks() {

	data_points.header.frame_id = params.frameid_map.c_str();
	data_points.points.resize(s.l.total);

	data_points.channels.resize(2);
	data_points.channels[0].name = "critical";
	data_points.channels[0].values.resize(s.l.total);
	
	data_points.channels[1].name = "covariance";
	data_points.channels[1].values.resize(s.l.total*9);

	int i, j;
	double inv_dep;	
	Vector3d m;	
	Vector6d yi;	
	Matrix<double,3,6> J;
	Matrix3d P;
	
	J.setZero();
	J(0,0) = J(1,1) = J(2,2) = 1.0;
	
	for(i=0; i<s.l.total; i++) {
	
		// -----------------
		// Observation Model
		// -----------------
		
		yi = s.x.segment<6>(sz+6*i);		
		inv_dep = 1.0/yi(5);

		double cya = cos(yi(3)),	cye = cos(yi(4));
		double sya = sin(yi(3)),	sye = sin(yi(4));
		
		m << cye*sya, -sye, cye*cya;	

		J.block<3,3>(0,3) << cye*cya,  -sye*sya, -cye*sya*inv_dep,
												 			 0,      -cye,      sye*inv_dep,
											  -cye*sya,  -sye*cya, -cye*cya*inv_dep;
	
		J.block<3,3>(0,3) *= inv_dep;

		P = J * s.P.block<6,6>(sz+6*i,sz+6*i) * J.transpose();
	
		for(j=0; j<9; j++)
			data_points.channels[1].values[j+9*i] = P(j/3,j%3);
		
		data_points.points[i].x = yi(0) + m(0)*inv_dep;
		data_points.points[i].y = yi(1) + m(1)*inv_dep;		
		data_points.points[i].z = yi(2) + m(2)*inv_dep;
		
		data_points.channels[0].values[i] = s.f(i);

	}
		
}

// ------------------------------------------------------------
//                    INITIALIZE STATE
// ------------------------------------------------------------

void Slam::initializeState() {

	// --------
	// Mean (x)
	// --------
	
	//ROS_INFO("Initialize: Starting...");	
	
	s.x.setZero(sz);
	s.x(3) = 1.0;
	if (!params.use_odo) 
		s.x(7)  = s.x(8)  = s.x(9)  = DBL_MIN;
	if (!params.use_imu)
		s.x(sz_w) = s.x(sz_w+1) = s.x(sz_w+2) = DBL_MIN;		
	
	s.x_odo = s.x.head<7>();
	
	// --------------
	// Covariance (P)
	// --------------

	s.P.setZero(sz,sz);
	s.P(0,0) = s.P(1,1) = s.P(2,2) = s.P(3,3) = s.P(4,4) = s.P(5,5) = s.P(6,6) = 0;
	if (!params.use_odo)
		s.P(7,7) = s.P(8,8) = s.P(9,9) = params.init_std_lin_vel*params.init_std_lin_vel;
	if (!params.use_imu)
		s.P(sz_w,sz_w) = s.P(sz_w+1,sz_w+1) = s.P(sz_w+2,sz_w+2) = params.init_std_ang_vel*params.init_std_ang_vel;	

	// ----------------------------
	// Landmarks (l) and Image (im)
	// ----------------------------

	s.l.total = 0;

	im.matched = 0;
	first_image = true;	
	got_image = false;

	// ----------
	// IMU (q_imu)
	// ----------
	
	if (params.use_imu) {	
		
		w_imu.x = 0.0;
		w_imu.y = 0.0;
		w_imu.z = 0.0;

		if (params.calibrate_imu) {
			std_srvs::Empty nada;
			ros::ServiceClient calibrate;
			calibrate = n.serviceClient<std_srvs::Empty>("/imu/calibrate");
			//ROS_INFO("Initialize: Calibrating IMU (10 seconds)...");
			calibrate.call(nada);
			//ROS_INFO("Initialize: Calibrating IMU - Done!");
		}

		first_imu  = true;
		update_imu = false;		
				
	}
	
	// --------
	// Odometry 
	// --------
	
	//if (params.use_odo) {		
		
		odo_dif.right = 0.0;	
		odo_dif.left  = 0.0;
		odo_arm = 0.0;	
		
		time_now_odo = time_prev_odo = 0.0;
		
		first_odo  = true;	
		update_odo = false;

	//}

	
	//ROS_INFO("Initialize: Done!");	
		
}


// ------------------------------------------------------------
//                        CALLBACKS
// ------------------------------------------------------------

void Slam::callbackOdo(const raposang_msgs::RaposaOdometry &odo_data) {

	double dt_odo;
	
	got_odo = true;

	//ROS_INFO("TIME: Odo - %f", odo_data.header.stamp.toSec());

	if(first_odo) {				
		//ROS_INFO("Data Received: First Odometry reading.");		
		
		odo_prev.right = odo_new.right = odo_data.track_right;
		odo_prev.left  = odo_new.left  = odo_data.track_left;
		time_prev_odo  = time_now_odo  = odo_data.header.stamp.toSec();
				
		first_odo = false;
	}
	else {
		
		if(update_odo) {
			odo_prev.right = odo_new.right;
			odo_prev.left  = odo_new.left;	
			time_prev_odo = time_now_odo;	
			update_odo = false;	
		}
		
		time_now_odo = odo_data.header.stamp.toSec();
		dt_odo = time_now_odo - time_prev_odo;		
		
		ROS_INFO("- ODOMETRY: %f seconds from last reading.", dt_odo);	
		odo_new.right = odo_data.track_right;
		odo_new.left  = odo_data.track_left;
		odo_dif.right = (odo_new.right - odo_prev.right);
		odo_dif.left  = (odo_new.left  - odo_prev.left );		
	}		
	
	if (params.arm_angle_always_zero)
		odo_arm = 0.0;
	else 
		odo_arm = (double) ( (VALUE_PI/180.0) * odo_data.arm_angle);
	
	//ROS_INFO("Data Received: Odometry: v(Left, Right) = %f %f Arm = %f", odo_prev.left, odo_prev.right, odo_arm);	
	
}

void Slam::callbackPublish(const ros::TimerEvent &event) {
	ROS_INFO("Data Published!");
	Slam::publish();
}

// ------------------------------------------------------------

void Slam::callbackIMU(const sensor_msgs::Imu &imu_data) {

	double dt_imu;

	if(params.use_gyro_only){

		w_imu.x = imu_data.angular_velocity.y;
		w_imu.y = -imu_data.angular_velocity.z;
		w_imu.z = -imu_data.angular_velocity.x;		
		
		
	}
	else {

		double do_w = imu_data.orientation.w;
		double do_x = imu_data.orientation.x;
		double do_y = imu_data.orientation.y;
		double do_z = imu_data.orientation.z;

		Quaterniond q(do_w, do_x, do_y, do_z);

		if (first_imu) {
		
			//ROS_INFO("Data Received: First IMU data reading.");		
		
			q_imu0_earth = q;		
					
			Matrix3d M_inew0_imu0;		
			
			M_inew0_imu0 << 0,  0, -1,
							  			1,  0,  0,
									  	0, -1,  0;
		
			q_inew0_imu0 = M_inew0_imu0;		
			q_imu_inew   = q_inew0_imu0.conjugate();		
														
			q_old.setIdentity(); 	
			q_new.setIdentity(); 	
			
			time_prev_imu  = time_now_imu  = imu_data.header.stamp.toSec();
			
			first_imu = false;		
			
		} else {

			Quaterniond q_imu;
			
			time_prev_imu = time_now_imu;	
			time_now_imu = imu_data.header.stamp.toSec();
			dt_imu = time_now_imu - time_prev_imu;	

			//ROS_INFO("Data Received: New IMU data, %f seconds from last reading.", dt_imu);

			q_old = q_new;
			
			q_new = q_imu_inew 
							* q.conjugate()	
							* q_imu0_earth 
							* q_inew0_imu0;

			q_imu = q_old * q_new.conjugate();

			q_imu.normalize();			
			
			float sign = (q_imu.w() < 0.0) ? -1.0 : 1.0;

			const double angle = 2.0 * acos(sign * q_imu.w());
			const double norm  = sqrt(1.0-q_imu.w()*q_imu.w());		
			const double vel   = angle / dt_imu;		
					
						
			if (norm > DBL_MIN) {
				w_imu.x = sign * q_imu.x()*vel/norm;
				w_imu.y = sign * q_imu.y()*vel/norm;
				w_imu.z = sign * q_imu.z()*vel/norm;	
			} else {
				w_imu.x = DBL_MIN;
				w_imu.y = DBL_MIN;
				w_imu.z = DBL_MIN;			
			}
			cout << w_imu.x << " " << w_imu.y << " " << w_imu.z << "\n\n";						
		}	
	}
	
	
	//ROS_INFO("Data Received: IMU: w(x,y,z) = %f %f %f", w_imu.x, w_imu.y, w_imu.z);	
			
}

// ------------------------------------------------------------


void Slam::callbackImage(const sensor_msgs::ImageConstPtr &img_msg) {

	// ------------------------
	// Subscription Information
	// ------------------------

	CvImagePtr cv_ptr = toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  
	cv::resize(cv_ptr->image, imleft, imleft.size(), 0, 0, cv::INTER_LINEAR);

	cv::cvtColor(imleft, imleft_graph, CV_GRAY2RGB);

	//ROS_INFO("TIME: Image Left - %f", img_msg->header.stamp.toSec());

	if(first_image) {
		//ROS_INFO("Data Received: First Image (L) reading!");	
		first_image = false;
	}
		//ROS_INFO("Data Received: New Image (L) reading!");
		
	got_image = true;

	// -----------------------
	// Get features from image
	// -----------------------

	Slam::extractFeatures();

}

void Slam::callbackFeatures(const raposang_msgs::RaposaFeatures &raposa_feat) {
	
	int i, k;

	if(first_image) 
		first_image = false;
	
	got_image = true;

	if (params.with_mono) 
		im.total = raposa_feat.stereo_features.size() + raposa_feat.left_features.size() 
							+ ((params.with_stereo) ? raposa_feat.right_features.size() : 0);
	else
		im.total = (params.with_stereo) ? raposa_feat.stereo_features.size() : 0;
		
	im.k.clear();
	im.d.create(im.total, 32, CV_8U);
	
	im.id_real.resize(im.total);
	im.id_real.setZero(im.total);		
	im.id_type.resize(im.total);
		
	ROS_INFO("- FETURES: %d features stereo, %d features left and %d features right", raposa_feat.stereo_features.size(), raposa_feat.left_features.size(), raposa_feat.right_features.size());
	
	k=0;
	
	if ((!params.with_mono && params.with_stereo) || params.with_mono) {	
	
		for(i=0; i<raposa_feat.stereo_features.size(); i++) {

			cv::circle(imleft_graph, cv::Point(raposa_feat.stereo_features[i].vl, raposa_feat.stereo_features[i].u), 2, CV_RGB(50,50,50), -1);	
			
			im.k.push_back(cv::KeyPoint(raposa_feat.stereo_features[i].vl, 
																	raposa_feat.stereo_features[i].u,
																	0.0,
																	raposa_feat.stereo_features[i].vr,
																	params.scale_stereo_str * raposa_feat.stereo_features[i].response));
										
			memcpy(&im.d.data[k*32], &raposa_feat.stereo_features[i].descriptor[0], 32);		
		//	//ROS_INFO("%f", raposa_feat.stereo_features[i].response);		
			im.id_type(k) = (params.with_stereo) ?  cam_stereo : cam_left ;	
			k++;
			
		}
	}
	
	if (params.with_mono) {

		for(i=0; i<raposa_feat.left_features.size(); i++) {
	
			cv::circle(imleft_graph, cv::Point(raposa_feat.left_features[i].v, raposa_feat.left_features[i].u), 2, CV_RGB(100,100,100), -1);	
			
			
			im.k.push_back(cv::KeyPoint(raposa_feat.left_features[i].v, 
																	raposa_feat.left_features[i].u,
																	0.0,
																	0.0,
																	raposa_feat.left_features[i].response));
										
			memcpy(&im.d.data[k*32], &raposa_feat.left_features[i].descriptor[0], 32);		
		//	//ROS_INFO("%f", raposa_feat.left_features[i].response*2.0);
			im.id_type(k) = cam_left;	
			k++;

		}

		if (params.with_stereo) {
			for(i=0; i<raposa_feat.right_features.size(); i++) {

				cv::circle(imleft_graph, cv::Point(raposa_feat.right_features[i].v, raposa_feat.right_features[i].u), 2, CV_RGB(100,100,100), -1);	
				
				
				im.k.push_back(cv::KeyPoint(raposa_feat.right_features[i].v, 
																		raposa_feat.right_features[i].u,
																		0.0,
																		0.0,
																		raposa_feat.right_features[i].response));
											
				memcpy(&im.d.data[k*32], &raposa_feat.right_features[i].descriptor[0], 32);		
				
				im.id_type(k) = cam_right;	
				k++;
				
				
			}
		}

	}
}

void Slam::extractFeatures() {

	int i;
	
	if(params.feature_detector == fd_surf) {		
		
		cv::SurfFeatureDetector surf_detector;
		surf_detector.detect(imleft, im.k);

		cv::SurfDescriptorExtractor extractor;
		extractor.compute(imleft, im.k, im.d);

	}	else if(params.feature_detector == fd_orb) {
		
		cv::ORB orb;
		orb(imleft, cv::Mat(), im.k, im.d);

	}
	
	im.total = im.k.size();
	im.id_real.resize(im.total);
	im.id_real.setZero(im.total);		
	im.id_type.resize(im.total);
	
	for(i=0; i<im.total; i++)
		im.id_type(i) = cam_left;		
}

void Slam::publish() {
	
	// -------------
	// Publish State
	// -------------

	//ROS_INFO("Display: Displaying state data...");
	//ROS_INFO("Display: r = %f %f %f   ", s.x(0), s.x(1), s.x(2));				
	//ROS_INFO("Display: q = %f %f %f %f", s.x(3), s.x(4), s.x(5), s.x(6));			
	//if (!params.use_odo)
		//ROS_INFO("Display: v = %f %f %f   ", s.x(7), s.x(8), s.x(9));
	//if (!params.use_imu)
		//ROS_INFO("Display: w = %f %f %f   ", s.x(sz_w), s.x(sz_w+1), s.x(sz_w+2));		
			
	raposang_msgs::RaposaSlam ngpose;			
	tf::Transform transform;	
		
	ngpose.header.stamp    = ros::Time::now();
	ngpose.header.frame_id = params.frameid_map.c_str();

	ngpose.time = got_image ? tic_tac : -1 ;
	ngpose.nr_landmarks = s.l.total;
	//ngpose.rem_landmarks = feat_removed;
	ngpose.rem_landmarks = feat_removed;
			
	ngpose.pose.position.x = s.x(0);
	ngpose.pose.position.y = s.x(1);		
	ngpose.pose.position.z = s.x(2);	
			
	ngpose.pose.orientation.w = s.x(3);
	ngpose.pose.orientation.x = s.x(4);		
	ngpose.pose.orientation.y = s.x(5);	
	ngpose.pose.orientation.z = s.x(6);

	ngpose.pose_odometry.position.x = s.x_odo(0);
	ngpose.pose_odometry.position.y = s.x_odo(1);		
	ngpose.pose_odometry.position.z = s.x_odo(2);	
			
	ngpose.pose_odometry.orientation.w = s.x_odo(3);
	ngpose.pose_odometry.orientation.x = s.x_odo(4);		
	ngpose.pose_odometry.orientation.y = s.x_odo(5);	
	ngpose.pose_odometry.orientation.z = s.x_odo(6);
	
	for(int i=0; i<9; i++)
		ngpose.covariance[i] = s.P(i/3,i%3);
		
	//cout << s.P.block<3,3>(0,0) << "\n\n";
		
	transform.setOrigin(tf::Vector3(ngpose.pose.position.x, 
																	ngpose.pose.position.y, 
																	ngpose.pose.position.z));
																	
	transform.setRotation(tf::Quaternion(ngpose.pose.orientation.x,
																		 	 ngpose.pose.orientation.y, 
																			 ngpose.pose.orientation.z,
																			 ngpose.pose.orientation.w));
																			 
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), params.frameid_map.c_str(), params.frameid_robot.c_str()));
	
	if(got_image) {		
		cv_bridge::CvImage cv_img;
		
	cv_img.header.frame_id = params.frameid_robot.c_str();	
	cv_img.encoding = sensor_msgs::image_encodings::RGB8;
	cv_img.image = imleft_graph;
	pubIL.publish(cv_img.toImageMsg()); 
	
	imleft_graph.setTo(0);
	
	Slam::writeLandmarks();	
						
	pub.publish(ngpose);
	pub_data.publish(data_points);

	//ROS_INFO("Display: Done!");

}

void Slam::rosSpin() {
	
	ros::Rate rate(params.reads_per_second);	
	
	Slam::initializeState();		
	
	while(ros::ok()) {	

		ros::spinOnce();
	
		if(!update_odo) {
				tic_tac = ros::Time::now().toSec();
				Slam::ekf(time_now_odo - time_prev_odo);						
				tic_tac = ros::Time::now().toSec() - tic_tac;
//			Slam::publish();	
				got_image = false;
		}
		
		rate.sleep();
	}
}

// ------------------------------------------------------------
//                           MAIN
// ------------------------------------------------------------

int main(int argc, char **argv) {

	ros::init(argc, argv, "raposang_slam");	
	ros::NodeHandle n;
	
	Slam raposa_slam(n);
	//ROS_INFO("RAPOSA-NG SLAM has started.");	
	raposa_slam.rosSpin();
	 
	return 0;

}

//EOF
