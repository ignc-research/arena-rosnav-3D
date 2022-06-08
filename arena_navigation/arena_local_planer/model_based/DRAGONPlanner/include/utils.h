#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Quadtree.h>
#include <sensor_msgs/LaserScan.h>

const float JACKAL_WIDTH = .48;
const float JACKAL_LENGTH = .56;

float proj(float theta){
	float div = (int)((theta+M_PI)/(2*M_PI));
	// ROS_INFO("theta is: %.2f\tdiv is: %.2f", theta, div);
	return theta-2*M_PI*div;
}


bool isPointCompletelyFree(const Eigen::Vector2f& p, const Eigen::Vector3f& pose, const sensor_msgs::LaserScan::ConstPtr& msg){
	Eigen::Vector2f vec(pose.x()-p.x(),pose.y()-p.y());
	float m = vec.y()/vec.x();
	float b = pose.y() - m*pose.x();

	for(int i = 0; i < msg->ranges.size(); i++){
	    float a= msg->angle_min + i*msg->angle_increment + pose.z();

	    Eigen::Vector2f p1(msg->ranges[i]*cos(a) + pose.x(), msg->ranges[i]*sin(a) + pose.y());
	    
	    float d = fabs(m*p1.x()-p1.y()+b)/sqrt(m*m+1);
	    if (d < JACKAL_WIDTH/2)
	    	return false;
	}

	return true;
}

float getAngleFromPoint(const Eigen::Vector3f& pose, const Eigen::Vector2f& point){
	float err = atan2(point.y()-pose.y(),point.x()-pose.x());
	if (err < 0)
	    err += 2*M_PI;

	// ROS_INFO("%.2f\t%.2f\t%.2f", err, pose.z(), err-pose.z());
	// err -= pose.z();

	return err;
}

int getIndicesFromPoint(const Eigen::Vector3f& pose, const sensor_msgs::LaserScan::ConstPtr& msg, const Eigen::Vector2f& point){
	float err = atan2(point.y()-pose.y(),point.x()-pose.x());
	if (err < 0)
	    err += 2*M_PI;

	// ROS_INFO("%.2f\t%.2f\t%.2f", err, pose.z(), err-pose.z());
	err -= pose.z();
	return  (int) (err/msg->angle_increment + msg->ranges.size()/2);
}

bool isGapNearScan(const Eigen::Vector3f& pose, const Eigen::Vector2f& gap, const sensor_msgs::LaserScan::ConstPtr& msg){
	int ind = getIndicesFromPoint(pose, msg, gap);

	for(int i = ind-10; i < ind+11; i++){
		if (ind < 0)
			continue;
		if (ind > msg->ranges.size()-1)
			break;

		float angle = msg->angle_min + i*msg->angle_increment + pose.z();
		Eigen::Vector2f p(msg->ranges[i]*cos(angle)+pose.x(),msg->ranges[i]*sin(angle)+pose.y());
        if ((gap-p).squaredNorm() < JACKAL_WIDTH*JACKAL_WIDTH/6){
        	// ROS_INFO("(%.2f, %.2f) is on scan, not making into gap", gap.x(), gap.y());
        	return true;
        }
	}

	return false;
}

float distToGap(const Eigen::Vector2f& ps, const Eigen::Vector2f& pe, const Eigen::Vector2f& p){
	Eigen::Vector2f gapVec = pe - ps;
	float d2 = gapVec.squaredNorm();

	double dot = (p-ps).dot(pe-ps);
	float t = std::max(0.0, std::min(1.0, dot/d2) );
	Eigen::Vector2f proj = ps + t*(pe-ps);
	return (proj-p).norm();
}

bool segmentGapIntersection(const Eigen::Vector4f& gap, const Eigen::Vector4f& segment){

    float a1 = gap.z()-gap.x();
    float b1 = -1*(segment.z()-segment.x());
    float c1 = segment.x()-gap.x();

    float a2 = gap.w()-gap.y();
    float b2 = -1*(segment.w()-segment.y());
    float c2 = segment.y()-gap.y();

    if (fabs(a1*b2-a2*b1) < 1e-6){
        float m1 = (gap.w()-gap.y())/(gap.z()-gap.x());
        float m2 = (segment.w()-segment.y())/(segment.z()-segment.x());

        float b1 = gap.y()-m1*gap.x();
        float b2 = segment.y()-m2*segment.x();

        return fabs(b1-b2) < 1e-3;
    }

    float s = (c1*b2-c2*b1)/(a1*b2-a2*b1);
    float t = (a1*c2-a2*c1)/(a1*b2-a2*b1);

    return 0 <= t && t <= 1 && 0 <= s && s <= 1;

}

bool getSegmentIntersection(const Eigen::Vector4f& gap, const Eigen::Vector4f& segment, Eigen::Vector2f& ret){
	
    float a1 = gap.z()-gap.x();
    float b1 = -1*(segment.z()-segment.x());
    float c1 = segment.x()-gap.x();

    float a2 = gap.w()-gap.y();
    float b2 = -1*(segment.w()-segment.y());
    float c2 = segment.y()-gap.y();

    if (fabs(a1*b2-a2*b1) < 1e-6){
        float m1 = (gap.w()-gap.y())/(gap.z()-gap.x());
        float m2 = (segment.w()-segment.y())/(segment.z()-segment.x());

        float b1 = gap.y()-m1*gap.x();
        float b2 = segment.y()-m2*segment.x();

        return fabs(b1-b2) < 1e-3;
    }

    float s = (c1*b2-c2*b1)/(a1*b2-a2*b1);
    float t = (a1*c2-a2*c1)/(a1*b2-a2*b1);

    if (0 <= t && t <= 1 && 0 <= s && s <= 1){
    	ret(0) = (gap.x() + (gap.z()-gap.x())*s);
    	ret(1) = (gap.y() + (gap.w()-gap.y())*s);
    	return false;
    }

    return true;
	
}

// https://www.geeksforgeeks.org/check-whether-two-points-x1-y1-and-x2-y2-lie-on-same-side-of-a-given-line-or-not/
bool pointsAreOnSameSideOfLine(float a, float b, float c, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {

    // float fx1 = a * p1.x() + b * p1.y() - c;
    // float fx2 = a * p2.x() + b * p2.y() - c;
 
    // // If fx1 and fx2 have same sign

    // ROS_INFO("fx1: %.2f\tfx2: %.2f",fx1,fx2);
    // if ((fx1 * fx2) > 0)
    //     return true;
 
    // return false;
    return ((p1.y() > a*p1.x()+c && p2.y() > a*p2.x()+c)
    	|| (p1.y() < a*p1.x()+c && p2.y() < a*p2.x()+c));
}

bool pointsAreOnSameSideOfLine(const Eigen::Vector3f& pose, const Eigen::Vector2f& p, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {

    float t1 = (pose.y()-p.y())*(p1.x()-pose.x())+(p.x()-pose.x())*(p1.y()-pose.y());
	float t2 = (pose.y()-p.y())*(p2.x()-pose.x())+(p.x()-pose.x())*(p2.y()-pose.y());
        	
 
    // If fx1 and fx2 have same sign
    if ((t1 * t2) > 0)
        return true;
 
    return false;
}


// bool isGapAdmissible(const Eigen::Vector2f& p, const sensor_msgs::LaserScan::ConstPtr& msg, const Eigen::Vector3f& pose){
	
// 	const float JACKAL_WIDTH = .48;
// 	const float JACKAL_LENGTH = .56;

// 	std::vector<Eigen::Vector2f> pointsA;
// 	std::vector<Eigen::Vector2f> pointsB;

// 	Eigen::Vector2f goalVec(p.x()-pose.x(), p.y()-pose.y());
// 	float goalD = goalVec.norm();

// 	float m = goalVec.y() / goalVec.x();
// 	float a = m;
// 	float b = -1;
// 	float c = pose.y()-m*pose.x();

// 	ROS_INFO("gap (%.2f, %.2f): equation is y=%.2f * x + %.2f", p.x(),p.y(),m, c);

// 	float aPerp = -1/m;
// 	float bPerp = -1;
// 	float cPerp = pose.x()/m + pose.y();

// 	ROS_INFO("gap (%.2f, %.2f): normal equation is y=%.2f * x + %.2f", p.x(),p.y(),aPerp, cPerp);

// 	Eigen::Vector2f roboPose(pose.x(),pose.y());
// 	for(int i = 0; i < msg->ranges.size(); i++){
// 		float angle= msg->angle_min + i*msg->angle_increment + pose.z();

//         Eigen::Vector2f p1(msg->ranges[i]*cos(angle) + pose.x(), msg->ranges[i]*sin(angle) + pose.y());
        
//         if (std::isinf(p1.x()) || std::isinf(p1.y()))
//         	continue;

//         if ((p-p1).norm() < JACKAL_WIDTH/2)
//         	return false;

//         if ((p1-roboPose).norm() > goalD)
//         	continue;

//         if (!pointsAreOnSameSideOfLine(aPerp,bPerp,cPerp,p1,p))
//         	continue;

//         float d = fabs(a*p1.x()+b*p1.y()+c)/sqrt(a*a+b*b);
//         if (d > JACKAL_WIDTH)
//         	continue;


//         // float side = a*p1.x()+b*p1.y()+c;
//         float side = (p1.x()-pose.x())*(p.y()-pose.y()) - (p1.y()-pose.y())*(p.x()-pose.x());

//         if (side <= 0){
//         	// ROS_INFO("A: (%.2f,%.2f)", p1.x(),p1.y());
//         	pointsA.push_back(p1);
//         }
//         else{
//         	// ROS_INFO("B: (%.2f,%.2f)", p1.x(),p1.y());
//         	pointsB.push_back(p1);
//         }
// 	}


// 	// ROS_INFO("A: %lu\tB: %lu",pointsA.size(), pointsB.size());
// 	for (Eigen::Vector2f pl : pointsA){

// 		for(Eigen::Vector2f pr : pointsB){
// 			// if ((pl-pr).norm() <= JACKAL_WIDTH){
// 			if ((pl-pr).norm() <= .65){
// 				// std::cout << pl << "--" << pr << "\t" << (pl-pr).norm() << std::endl;
// 				ROS_INFO("(%.2f,%.2f) too close to (%.2f, %.2f)", pl.x(), pl.y(),pr.x(),pr.y());
// 				return false;
// 			}
// 		}

// 	}

// 	return true;


// }

bool isGapAdmissible(const Eigen::Vector2f& p, const sensor_msgs::LaserScan::ConstPtr& msg, const Eigen::Vector3f& pose){
	
	const float JACKAL_WIDTH = .48;
	const float JACKAL_LENGTH = .56;

	float pAngle = getAngleFromPoint(pose, p);

	std::vector<Eigen::Vector2f> pointsA;
	std::vector<Eigen::Vector2f> pointsB;

	Eigen::Vector2f goalVec(p.x()-pose.x(), p.y()-pose.y());
	float goalD = goalVec.norm();

	float m = goalVec.y() / goalVec.x();
	float a = m;
	float b = -1;
	float c = pose.y()-m*pose.x();

	// if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
		// ROS_INFO("gap (%.2f, %.2f): equation is y=%.2f * x + %.2f", p.x(),p.y(),m, c);

	float aPerp = -1/m;
	float bPerp = -1;
	float cPerp = pose.x()/m + pose.y();

	// if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
		// ROS_INFO("gap (%.2f, %.2f): normal equation is y=%.2f * x + %.2f", p.x(),p.y(),aPerp, cPerp);

	Eigen::Vector2f roboPose(pose.x(),pose.y());
	Eigen::Vector2f firstPoint;

	int infcount = 0;
	for(int i = 0; i < msg->ranges.size(); i++){
		float angle= msg->angle_min + i*msg->angle_increment + pose.z();

        Eigen::Vector2f p1(msg->ranges[i]*cos(angle) + pose.x(), msg->ranges[i]*sin(angle) + pose.y());
        
        if (std::isinf(p1.x()) || std::isinf(p1.y()))
        	continue;

        if ((p-p1).norm() < JACKAL_WIDTH/2)
        	return false;

        if (fabs(pAngle-pose.z())>M_PI && (p1-roboPose).squaredNorm() < JACKAL_WIDTH*JACKAL_WIDTH/4 + JACKAL_LENGTH*JACKAL_LENGTH/4)
        	return false;

        if ((p1-roboPose).norm() > goalD)
        	continue;

        // if (!pointsAreOnSameSideOfLine(aPerp,bPerp,cPerp,p1,p))
        // 	continue;
        if (!pointsAreOnSameSideOfLine(aPerp,bPerp,cPerp,p1,p)){
    //     	if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
				// ROS_INFO("\t(%.2f, %.2f) and (%.2f, %.2f) are on opposite sides",
				//  p.x(),p.y(),p1.x(),p1.y());
        	continue;
        }
    //     else{
    //     	if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
				// ROS_INFO("\t(%.2f, %.2f) and (%.2f, %.2f) are on same sides",
				//  p.x(),p.y(),p1.x(),p1.y());
    //     }

        float d = fabs(a*p1.x()+b*p1.y()+c)/sqrt(a*a+b*b);
        if (d > JACKAL_WIDTH)
        	continue;


        // float side = a*p1.x()+b*p1.y()+c;
        if (pointsA.size() == 0 && pointsB.size() == 0){
        	pointsA.push_back(p1);
        	firstPoint = p1;
        }
        else{
        	float t1 = (pose.y()-p.y())*(firstPoint.x()-pose.x())+(p.x()-pose.x())*(firstPoint.y()-pose.y());
        	float t2 = (pose.y()-p.y())*(p1.x()-pose.x())+(p.x()-pose.x())*(p1.y()-pose.y());
        	if (t1*t2 < 0)
        		pointsB.push_back(p1);
        	else
        		pointsA.push_back(p1);
        }

	}


	// ROS_INFO("A: %lu\tB: %lu",pointsA.size(), pointsB.size());
	for (Eigen::Vector2f pl : pointsA){

		for(Eigen::Vector2f pr : pointsB){
			// if ((pl-pr).norm() <= JACKAL_WIDTH){
			if ((pl-pr).norm() <= .57){
				// std::cout << pl << "--" << pr << "\t" << (pl-pr).norm() << std::endl;
				// if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
				// 	ROS_INFO("(%.2f,%.2f) too close to (%.2f, %.2f)", pl.x(), pl.y(),pr.x(),pr.y());
				
				// if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
				// 	exit(0);

				return false;
			}
		}

	}

	// if(fabs(p.x()) < 1e-1 && fabs(p.y()-10) < 1e-1)
	// 	exit(0);

	return true;

}

bool isGapAdmissible(const Eigen::Vector2f& p, const Eigen::Vector2f& q,const sensor_msgs::LaserScan::ConstPtr& msg, const Eigen::Vector3f& pose){
	const float JACKAL_WIDTH = .48;
	const float JACKAL_LENGTH = .56;

	std::vector<Eigen::Vector2f> pointsA;
	std::vector<Eigen::Vector2f> pointsB;

	Eigen::Vector2f goalVec(p.x()-q.x(), p.y()-q.y());
	float goalD = goalVec.norm();

	float m = goalVec.y() / goalVec.x();
	float a = m;
	float b = -1;
	float c = q.y()-m*q.x();

	// ROS_INFO("gap (%.2f, %.2f): equation is y=%.2f * x + %.2f", p.x(),p.y(),m, c);

	float aPerp = -1/m;
	float bPerp = -1;
	float cPerp = q.x()/m + q.y();
	Eigen::Vector2f firstPoint;

	for(int i = 0; i < msg->ranges.size(); i++){
		float angle= msg->angle_min + i*msg->angle_increment + pose.z();

        Eigen::Vector2f p1(msg->ranges[i]*cos(angle) + pose.x(), msg->ranges[i]*sin(angle) + pose.y());
        
        if (std::isinf(p1.x()) || std::isinf(p1.y()))
        	continue;

        if ((p-p1).norm() < JACKAL_WIDTH/2)
        	return false;

        if ((q-p1).norm() > goalD)
        	continue;

        if (!pointsAreOnSameSideOfLine(aPerp,bPerp,cPerp,p1,p))
        	continue;

        float d = fabs(a*p1.x()+b*p1.y()+c)/sqrt(a*a+b*b);
        if (d > JACKAL_WIDTH)
        	continue;


        // float side = a*p1.x()+b*p1.y()+c;
        if (pointsA.size() == 0 && pointsB.size() == 0){
        	pointsA.push_back(p1);
        	firstPoint = p1;
        }
        else{
        	float t1 = (pose.y()-p.y())*(firstPoint.x()-pose.x())+(p.x()-pose.x())*(firstPoint.y()-pose.y());
        	float t2 = (pose.y()-p.y())*(p1.x()-pose.x())+(p.x()-pose.x())*(p1.y()-pose.y());
        	if (t1*t2 < 0)
        		pointsB.push_back(p1);
        	else
        		pointsA.push_back(p1);
        }
	}


	// ROS_INFO("A: %lu\tB: %lu",pointsA.size(), pointsB.size());
	for (Eigen::Vector2f pl : pointsA){

		for(Eigen::Vector2f pr : pointsB){
			// if ((pl-pr).norm() <= JACKAL_WIDTH){
			if ((pl-pr).norm() <= .57){
				// std::cout << pl << "--" << pr << "\t" << (pl-pr).norm() << std::endl;
				// ROS_INFO("(%.2f,%.2f) too close to (%.2f, %.2f)", pl.x(), pl.y(),pr.x(),pr.y());
				return false;
			}
		}

	}

	return true;


}

std::vector<Eigen::Vector2f> findVirtualGap(const Eigen::Vector3f& pose, const quadtree::Node& currGap, const sensor_msgs::LaserScan::ConstPtr& msg){

	std::vector<Eigen::Vector2f> virtualGap;
	virtualGap.push_back(currGap.ps);
	virtualGap.push_back(currGap.pe);

	int count = 0;

	while(count++ < 100){
		std::vector<Eigen::Vector2f> Oin;
		std::vector<Eigen::Vector2f> Oex;

		Eigen::Vector2f pl;
		Eigen::Vector2f pr;

		float thetaL = getAngleFromPoint(pose, virtualGap[0]);
		float thetaR = getAngleFromPoint(pose, virtualGap[1]);

		if (thetaR > thetaL){
			float tmp = thetaL;
			thetaL = thetaR;
			thetaR = tmp;
			pl = virtualGap[1];
			pr = virtualGap[0];
		} else{
			pl = virtualGap[0];
			pr = virtualGap[1];
		}

		Eigen::Vector2f mp = (pl+pr)/2;
		Eigen::Vector2f vec(pose.x()-mp.x(),pose.y()-mp.y());
		float m = vec.y()/vec.x();
		float b = pose.y() - m*pose.x();

		for(int i = 0; i < msg->ranges.size(); i++){

			if (msg->ranges[i] > msg->range_max)
				continue;

			float angle = msg->angle_min + i*msg->angle_increment + pose.z();
			// if (fabs(angle-pl) > M_PI || fabs(angle-pr > M_PI))
			// 	continue;

			Eigen::Vector2f p(msg->ranges[i]*cos(angle) + pose.x(), msg->ranges[i]*sin(angle) + pose.y());

			if (angle > thetaR && angle < thetaL){
				float projThetaR = proj(angle-thetaR);
				float projThetaL = proj(angle-thetaL);
				// ROS_INFO("err is %.2f\tproj(err) is  %.2f", angle-thetaR, projThetaR);
				if (projThetaR > 0)
					Oex.push_back(p);
				if (projThetaL < 0)
					Oex.push_back(p);
			}

			else
				Oin.push_back(p);
		}

		std::vector<Eigen::Vector2f> OexCollision;
		Eigen::Vector2f champ;
		float champD = 1000;
		for (Eigen::Vector2f p : Oex){
			float d = fabs(m*p.x()-p.y()+b)/sqrt(m*m+1);
			if(d < JACKAL_WIDTH/2){
				if (champD > d){
					champ = p;
					champD = d;
				}

				OexCollision.push_back(p);
			}
		}

		if (OexCollision.size() == 0){
			bool isValid = true;
			for (Eigen::Vector2f p : Oin){
				float d = fabs(m*p.x()-p.y()+b)/sqrt(m*m+1);
				if(d < JACKAL_WIDTH/2){
					isValid = false;
					break;
				}
			}

			if (isValid){
				return virtualGap;
			}
			else{
				virtualGap.clear();
				return virtualGap;
			}
		}

		Eigen::Vector2f pf = champ;
		champD = 1000;
		Eigen::Vector2f ps;
		for(Eigen::Vector2f p : Oex){
			if (!pointsAreOnSameSideOfLine(m,-1,b, pf, p)){
				float d = (p-pf).squaredNorm();
				if (d < champD){
					champD = d;
					ps = p;
				}
			}
		}

		virtualGap[0] = ps;
		virtualGap[1] = pf;
	}

	virtualGap.clear();
	return virtualGap;


}

#endif
