/* 
 
 * Configuration of the UR robot
 *
 * Conf 0 = shoulder right, elbow up, wrist up
 * Conf 1 = shoulder right, elbow down, wrist up
 * Conf 2 = shoulder right, elbow up, wrist down
 * Conf 3 = shoulder right, elbow down, wrist down
 * Conf 4 = shoulder left, elbow down, wrist down
 * Conf 5 = shoulder left, elbow up, wrist down
 * Conf 6 = shoulder left, elbow down, wrist up
 * Conf 7 = shoulder left, elbow up, wrist up
 * 
 * Singular Configuration: q5 = 0 or q5 = pi
 * 
 */

#include "ik_ur.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inverse_kinem{

    FIKServer::FIKServer(int rate) : Node("ik_ur")
    {
        this->rate = rate;
        
        this->initialiseRos();

        std::cout<<"RATE Kine: "<<rate<<std::endl;

    }

    FIKServer::~FIKServer()
    {
    }

    void FIKServer::coefficients()
    {   
        std::cout<<"UR Type: "<<this->ur_type<<std::endl;
        if(this->ur_type == "UR5e" or this->ur_type == "ur5e")
        {
            this->a2 = -0.4253630103489888;
            this->a3 = -0.3924349641369202;

            this->d1 =  0.162710833731481;
            this->d4 =  0.1337873018461449;
            this->d5 =  0.09960901169401305;
            this->d6 =  0.09949603061275286;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "UR5"  or this->ur_type == "ur5")
        {
            this->a2 = -0.4255204973093404;
            this->a3 = -0.3920641883612647;

            this->d1 =  0.08944066670827282;
            this->d4 =  0.1109001909756448;
            this->d5 =  0.09482875198638333;
            this->d6 =  0.08256209008760497;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "UR10e"  or this->ur_type == "ur10e")
        {
            this->a2 = -0.6127;
            this->a3 = -0.57155;

            this->d1 =  0.1807;
            this->d4 =  0.17415;
            this->d5 =  0.11985;
            this->d6 =  0.11655;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "UR10e_left" or this->ur_type == "ur10e_left")
        {
            this->a2 = -0.6119964323534905;
            this->a3 = -0.5710503060002635;

            this->d1 =  0.1806927933190718;
            this->d4 =  0.1741796604938954;
            this->d5 =  0.1197842415354139;
            this->d6 =  0.1154690142299182;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "UR10e_right" or this->ur_type == "ur10e_right")
        {
            this->a2 = -0.6120611916228695;
            this->a3 = -0.5711677680280721;

            this->d1 =  0.1806621104011266;
            this->d4 =  0.1739960120965293;
            this->d5 =  0.1196669865329291;
            this->d6 =  0.1154645510896043;
           
   
            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "TM900" or this->ur_type == "tm900")
        {
            this->a2 = -0.4290;
            this->a3 = -0.4115;

            this->d1 =  0.1451;
            this->d4 =  0.1222;
            this->d5 =  0.1060;
            this->d6 =  0.1144;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else
        {   
            RCLCPP_ERROR(this->get_logger(), "Invalid UR type");
        }

        //correzione per il sistema di riferimento

        //matrice di rotazione di 90 gradi attorno all'asse x
        Eigen::AngleAxisd rot_x(M_PI/2, Eigen::Vector3d::UnitX());
        Rx_fk = rot_x.toRotationMatrix();

        //matrice di rotazione di 90 gradi attorno all'asse z
        Eigen::AngleAxisd rot_z(M_PI/2, Eigen::Vector3d::UnitZ());
        Rz_fk = rot_z.toRotationMatrix();
    }

    void FIKServer::initialiseRos()
    {
        this->fk_service = create_service<message::srv::UrForwardKinematics>("ur_forward_k", std::bind(&FIKServer::fk_cb, this, std::placeholders::_1, std::placeholders::_2));
        this->ik_service = create_service<message::srv::UrInverseKinematics>("ur_inverse_k", std::bind(&FIKServer::ik_cb, this, std::placeholders::_1, std::placeholders::_2));

        std::cout<<"Kinematics Service Started"<<std::endl;
    }

    // //      FORWARD KINEMATICS

	Eigen::Matrix4d FIKServer::AH(double alpha, double a, double d, double theta)
	{
		Eigen::Matrix4d A;
		A <<    std::cos(theta),    -std::sin(theta)*std::cos(alpha),     std::sin(theta)*std::sin(alpha),      a*std::cos(theta),
                std::sin(theta),     std::cos(theta)*std::cos(alpha),     -std::cos(theta)*std::sin(alpha),     a*std::sin(theta),
                0,                   std::sin(alpha),                      std::cos(alpha),                     d,
                0,                   0,                                     0,                                  1;

		return A;
	}

    
	Eigen::Matrix4d FIKServer::HTrans(std::vector<double> q)
	{
        Eigen::Matrix4d T_0_1 = AH(this->alph1,    0,          this->d1,   q[0]);
        Eigen::Matrix4d T_1_2 = AH(0,              this->a2,   0,          q[1]);
        Eigen::Matrix4d T_2_3 = AH(0,              this->a3,   0,          q[2]);
        Eigen::Matrix4d T_3_4 = AH(this->alph4,    0,          this->d4,   q[3]);
        Eigen::Matrix4d T_4_5 = AH(this->alph5,    0,          this->d5,   q[4]);
        Eigen::Matrix4d T_5_6 = AH(0,              0,          this->d6,   q[5]);

        Eigen::Matrix4d T_0_6 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6;

        T_0_6.block<3,3>(0,0) = T_0_6.block<3,3>(0,0)*Rx_fk;
        T_0_6.block<3,3>(0,0) = T_0_6.block<3,3>(0,0)*Rz_fk;

        return T_0_6;
	}


    bool FIKServer::fk_cb(const std::shared_ptr<message::srv::UrForwardKinematics::Request> req, std::shared_ptr<message::srv::UrForwardKinematics::Response> res)
    {
        this->ur_type = req->ur_type;
        this->coefficients();

        res->reference_pose.resize(req->reference_joints.size());

        for (unsigned int i = 0; i < req->reference_joints.size(); i++)
        {
            if(this->ur_type == "TM900" or this->ur_type == "tm900")
            {
                req->reference_joints[i].data[1] = req->reference_joints[i].data[1] + M_PI/2;
                req->reference_joints[i].data[3] = req->reference_joints[i].data[3] + M_PI/2;

                req->reference_joints[i].data[1] = -req->reference_joints[i].data[1];
                req->reference_joints[i].data[2] = -req->reference_joints[i].data[2];
                req->reference_joints[i].data[3] = -req->reference_joints[i].data[3];
            }
            
            //create input vector from request
            std::vector<double> q {req->reference_joints[i].data[0], req->reference_joints[i].data[1], req->reference_joints[i].data[2], req->reference_joints[i].data[3], req->reference_joints[i].data[4], req->reference_joints[i].data[5]};
            
            //compute solution
            Eigen::Matrix4d T_0_6 = this->HTrans(q); 
            
            //generate response message

            res->reference_pose[i].position.x = T_0_6(0,3);
            res->reference_pose[i].position.y = T_0_6(1,3);
            res->reference_pose[i].position.z = T_0_6(2,3);
            
            Eigen::Matrix3d rot = T_0_6.block<3,3>(0,0);

            Eigen::Quaterniond quat(rot);

            res->reference_pose[i].orientation.x = quat.x();
            res->reference_pose[i].orientation.y = quat.y();
            res->reference_pose[i].orientation.z = quat.z();
            res->reference_pose[i].orientation.w = quat.w();
        }

        return true;         
    }

    //      INVERSE KINEMATICS
    Eigen::Matrix<double,8,6> FIKServer::invKine(Eigen::Matrix4d desired_pos)
    {   
        Eigen::Matrix<double,8,6> ik_solutions;
        
        //calcolo theta1

        for (int i = 0; i < 4; i++)
        {
            ik_solutions(i,0) = std::atan2(this->d4, std::sqrt(std::pow((this->d6*desired_pos(1,2) - desired_pos(1,3)),2) + std::pow((desired_pos(0,3)-this->d6*desired_pos(0,2)),2) - std::pow(this->d4,2))) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            if(std::isnan(ik_solutions(i,0)))
            {
                ik_solutions(i,0) = std::atan2(this->d4, 0) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            }
            ik_solutions(i+4,0) = std::atan2(this->d4, - std::sqrt(std::pow((this->d6*desired_pos(1,2) - desired_pos(1,3)),2) + std::pow((desired_pos(0,3)-this->d6*desired_pos(0,2)),2) - std::pow(this->d4,2))) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            if(std::isnan(ik_solutions(i+4,0)))
            {
                ik_solutions(i+4,0) = std::atan2(this->d4, 0) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            }
        }
        //calcolo theta5

        for (int i = 0; i < 2; i++)
        {
            ik_solutions(i,4) = std::atan2(std::sqrt(std::pow((desired_pos(0,0)*std::sin(ik_solutions(i,0)) - desired_pos(1,0)*std::cos(ik_solutions(i,0))),2) + std::pow(desired_pos(0,1)*std::sin(ik_solutions(i,0)) - desired_pos(1,1)*std::cos(ik_solutions(i,0)),2)), desired_pos(0,2)*std::sin(ik_solutions(i,0)) - desired_pos(1,2)*std::cos(ik_solutions(i,0)));

            ik_solutions(i+2,4) = std::atan2( - std::sqrt(std::pow((desired_pos(0,0)*std::sin(ik_solutions(i+2,0)) - desired_pos(1,0)*std::cos(ik_solutions(i+2,0))),2) + std::pow(desired_pos(0,1)*std::sin(ik_solutions(i+2,0)) - desired_pos(1,1)*std::cos(ik_solutions(i+2,0)),2)), desired_pos(0,2)*std::sin(ik_solutions(i+2,0)) - desired_pos(1,2)*std::cos(ik_solutions(i+2,0)));

            ik_solutions(i+4,4) = std::atan2(std::sqrt(std::pow((desired_pos(0,0)*std::sin(ik_solutions(i+4,0)) - desired_pos(1,0)*std::cos(ik_solutions(i+4,0))),2) + std::pow(desired_pos(0,1)*std::sin(ik_solutions(i+4,0)) - desired_pos(1,1)*std::cos(ik_solutions(i+4,0)),2)), desired_pos(0,2)*std::sin(ik_solutions(i+4,0)) - desired_pos(1,2)*std::cos(ik_solutions(i+4,0)));

            ik_solutions(i+4+2,4) = std::atan2( - std::sqrt(std::pow((desired_pos(0,0)*std::sin(ik_solutions(i+2+4,0)) - desired_pos(1,0)*std::cos(ik_solutions(i+2+4,0))),2) + std::pow(desired_pos(0,1)*std::sin(ik_solutions(i+2+4,0)) - desired_pos(1,1)*std::cos(ik_solutions(i+2+4,0)),2)), desired_pos(0,2)*std::sin(ik_solutions(i+2+4,0)) - desired_pos(1,2)*std::cos(ik_solutions(i+2+4,0)));
        }

        //calcolo theta6

        for (int i = 0; i < 8; i++)
        {   
            if ((ik_solutions(i,4)) != 0 and (ik_solutions(i,4)) != M_PI)
            {
                ik_solutions(i,5) = std::atan2(-(desired_pos(0,1)*std::sin(ik_solutions(i,0)) - desired_pos(1,1)*std::cos(ik_solutions(i,0)))/std::sin(ik_solutions(i,4)), (desired_pos(0,0)*std::sin(ik_solutions(i,0)) - desired_pos(1,0)*std::cos(ik_solutions(i,0)))/std::sin(ik_solutions(i,4)));
            }
            else
            {
                std::cout << "Theta 6 not defined" << std::endl;
                std::cout << "Singular Configuration " << std::endl;
            }
            
        }


        //calcolo theta2, theta3 e theta4

        for (int i = 0; i < 8; i++)
        {   
            if ((ik_solutions(i,4)) != 0 and (ik_solutions(i,4)) != M_PI)
            {
                //calcolo theta234
                double theta234 = std::atan2(- desired_pos(2,2)/std::sin(ik_solutions(i,4)), - (desired_pos(0,2)*std::cos(ik_solutions(i,0)) + desired_pos(1,2)*std::sin(ik_solutions(i,0)))/std::sin(ik_solutions(i,4)));

                //calcolo theta2
                double A = desired_pos(0,3)*std::cos(ik_solutions(i,0)) + desired_pos(1,3)*std::sin(ik_solutions(i,0)) - this->d5*std::sin(theta234) + this->d6*std::sin(ik_solutions(i,4))*std::cos(theta234);
                
                double B = desired_pos(2,3) - this->d1 + this->d5*std::cos(theta234) + this->d6*std::sin(ik_solutions(i,4))*std::sin(theta234);

                double C = 2*this->a2*std::sqrt(std::pow(A,2) + std::pow(B,2));

                if (i%2 == 0)
                {
                    ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, std::sqrt(1-std::pow((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C ,2))) - std::atan2(A,B);
 
                    if (std::isnan(ik_solutions(i,1)) and !std::isnan(theta234))
                    {
                        ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, 0) - std::atan2(A,B);
                    }       
                }
                else
                {
                    ik_solutions(i,1) = std::atan2((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, - std::sqrt(1-std::pow((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C ,2))) - std::atan2(A,B);
                    
                    if (std::isnan(ik_solutions(i,1)) and !std::isnan(theta234))
                    {
                        ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, 0) - std::atan2(A,B);
                    }
                    
                }

                //calcolo theta23
                double theta23 = std::atan2((desired_pos(2,3)-this->d1+this->d5*std::cos(theta234)-this->a2*std::sin(ik_solutions(i,1))+this->d6*std::sin(ik_solutions(i,4))*std::sin(theta234))/this->a3, (desired_pos(0,3)*std::cos(ik_solutions(i,0))+desired_pos(1,3)*std::sin(ik_solutions(i,0))-this->d5*std::sin(theta234)-this->a2*std::cos(ik_solutions(i,1))+this->d6*std::sin(ik_solutions(i,4))*std::cos(theta234))/this->a3);
                
                //calcolo theta3
                ik_solutions(i,2) = theta23 - ik_solutions(i,1);

                //calcolo theta4
                ik_solutions(i,3) = theta234 - ik_solutions(i,2) - ik_solutions(i,1);
                
            }
            else
            {
                std::cout << "Theta 2, 3 and 4 not defined" << std::endl;
                std::cout << "Singular Configuration " << std::endl;
            }
        }

        //riporto i risulatati nell'intervallo [-pi,pi]
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (ik_solutions(i,j) > M_PI)
                {
                    ik_solutions(i,j) = ik_solutions(i,j) - 2*M_PI;
                }
                else if (ik_solutions(i,j) < -M_PI)
                {
                    ik_solutions(i,j) = ik_solutions(i,j) + 2*M_PI;
                }
            }
        }

        // chiamo la cinematica diretta per verificare che le soluzioni siano corrette
        std::vector<double> q(6);
        Eigen::Matrix4d T_0_6;
        double max_single_error = 0;

        for (int i = 0; i < 8; i++) 
        {
            // if(this->ur_type == "TM900" or this->ur_type == "tm900")
            // {
            //     q = {ik_solutions(i, 0), -(ik_solutions(i, 1) + M_PI/2), - ik_solutions(i, 2), -(ik_solutions(i, 3) + M_PI/2), ik_solutions(i, 4), ik_solutions(i, 5)};
            // }
            // else
            // {
            q = {ik_solutions(i, 0), ik_solutions(i, 1), ik_solutions(i, 2), ik_solutions(i, 3), ik_solutions(i, 4), ik_solutions(i, 5)};
            // }
            
            T_0_6 = this->HTrans(q);
            
            // calcolo distanza euclidea tra la posizione desiderata e quella calcolata
            max_single_error = std::sqrt(std::pow((desired_pos(0,3) - T_0_6(0,3)),2) + std::pow((desired_pos(1,3) - T_0_6(1,3)),2) + std::pow((desired_pos(2,3) - T_0_6(2,3)),2));

            if(max_single_error > this->max_error[i]) 
            {
                this->max_error[i] = max_single_error;
            } 

        }


        // std::cout << "IK solutions: " << std::endl;
        // std::cout << ik_solutions << std::endl;

        return ik_solutions;
    }


    bool FIKServer::ik_cb(const std::shared_ptr<message::srv::UrInverseKinematics::Request> req, std::shared_ptr<message::srv::UrInverseKinematics::Response> res)
    {   


        this->max_error = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        this->verbose = req->verbose;

        std::vector<int> desired_config = req->desired_config;
        // ordina il vettore desired config in ordine crescente
        std::sort(desired_config.begin(), desired_config.end());

        // DH parameters
        this->ur_type = req->ur_type;
        this->coefficients();

        // correzioni per il mio sistema di riferimento

        //matrice di rotazione di -90 gradi attorno all'asse x
        Eigen::AngleAxisd rot_x(-M_PI/2, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d Rx = rot_x.toRotationMatrix();

        //matrice di rotazione di -90 gradi attorno all'asse z
        Eigen::AngleAxisd rot_z(-M_PI/2, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d Rz = rot_z.toRotationMatrix();

        // vettore di quaternioni
        std::vector<Eigen::Quaterniond> q;
        
        // fill the vector
        for (int i = 0; i < req->reference_pose.size(); i++)
        {
            Eigen::Quaterniond temp;
            temp.x() = req->reference_pose[i].orientation.x;
            temp.y() = req->reference_pose[i].orientation.y;
            temp.z() = req->reference_pose[i].orientation.z;
            temp.w() = req->reference_pose[i].orientation.w;
            q.push_back(temp);
        }
        // vettore di matrici di rotazione
        std::vector<Eigen::Matrix3d> R;

        // fill the vector
        for (int i = 0; i < q.size(); i++)
        {
            Eigen::Matrix3d temp = q[i].toRotationMatrix();
            R.push_back(temp);
        }

        // vettore di omonogenee di riferimento
        std::vector<Eigen::Matrix4d> reference;

        // fill the vector
        for (int i = 0; i < R.size(); i++)
        {
            Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
            for(int j = 0; j<3 ;j++){
                for(int k = 0; k<3 ;k++){
                    temp(j,k) = R[i](j,k);
                }
            }
            temp(0,3) = req->reference_pose[i].position.x;
            temp(1,3) = req->reference_pose[i].position.y;
            temp(2,3) = req->reference_pose[i].position.z;
            reference.push_back(temp);
        }


        res->success = {true, true, true, true, true, true, true, true};
        //inizializzo la matrice di soluzioni
        this->res_new.complete_solution.resize(reference.size());
        this->res_new.solution.resize(reference.size());

        unsigned int iteratore = 0;
        Eigen::Matrix<double,8,6> ik_solutions;

        for (int i = 0; i < reference.size(); i++)
        {
            //correzione per il mio sistema di riferimento1
            reference[i].block<3,3>(0,0) = reference[i].block<3,3>(0,0) * Rz;
            reference[i].block<3,3>(0,0) = reference[i].block<3,3>(0,0) * Rx;
            //calcolo la IK
            ik_solutions = this->invKine(reference[i]);

            //resize
            this->res_new.complete_solution[i].joint_matrix.resize(8);
            this->res_new.solution[i].joint_matrix.resize(desired_config.size());

            //riempio la matrice di soluzioni
            for (int j = 0; j < 8; j++)
            {
                this->res_new.complete_solution[i].joint_matrix[j].data.resize(6);
                for (int k = 0; k < 6; k++)
                {
                    this->res_new.complete_solution[i].joint_matrix[j].data[k] = ik_solutions(j,k);

                    if (j == desired_config[iteratore] and iteratore < desired_config.size())
                    {
                        this->res_new.solution[i].joint_matrix[iteratore].data.resize(6);
                        this->res_new.solution[i].joint_matrix[iteratore].data[k] = ik_solutions(j,k);
                        
                        iteratore++;
                    }
                }
            }

            //resetto l'iteratore
            iteratore = 0;
        }


        this->res_new.move_q6 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // chiamo funzione di controllo per l'ultimo giunto
        if(req->check_q6)
        {
            if(req->last_joints.size() == 0 or req->last_joints.size() != 6)
            {
                this->last_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
            else
            {
                this->last_joints = {req->last_joints[0], req->last_joints[1], req->last_joints[2], req->last_joints[3], req->last_joints[4], req->last_joints[5]};
            }
            
            this->checkq6();
        }

        // check the joint limits
        iteratore = 0;
        
        for (int i = 0; i < this->res_new.complete_solution.size(); i++)
        {
            for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
            {
                int count = 0;

                if(this->ur_type == "TM900" or this->ur_type == "tm900")
                {
                    this->res_new.complete_solution[i].joint_matrix[j].data[1] = -this->res_new.complete_solution[i].joint_matrix[j].data[1];
                    this->res_new.complete_solution[i].joint_matrix[j].data[2] = -this->res_new.complete_solution[i].joint_matrix[j].data[2];
                    this->res_new.complete_solution[i].joint_matrix[j].data[3] = -this->res_new.complete_solution[i].joint_matrix[j].data[3];

                    this->res_new.complete_solution[i].joint_matrix[j].data[1] = this->res_new.complete_solution[i].joint_matrix[j].data[1] - M_PI/2;
                    this->res_new.complete_solution[i].joint_matrix[j].data[3] = this->res_new.complete_solution[i].joint_matrix[j].data[3] - M_PI/2;
                }

                if (j == desired_config[iteratore] and iteratore < desired_config.size())
                {
                    this->res_new.solution[i].joint_matrix[iteratore].data = this->res_new.complete_solution[i].joint_matrix[j].data;
                        
                    iteratore++;
                }

                for (int k = 0; k < this->res_new.complete_solution[i].joint_matrix[j].data.size(); k++)
                {
                        
                    if (this->res_new.complete_solution[i].joint_matrix[j].data[k] < this->joint_limits[count] or this->res_new.complete_solution[i].joint_matrix[j].data[k] > this->joint_limits[count+1])
                    {
                        if(not (this->ur_type == "TM900" or this->ur_type == "tm900"))
                        {
                            res->success[j] = false;
                            if(this->verbose)
                            {   
                                RCLCPP_ERROR(this->get_logger(), "Joint limits exceeded!");
                                std::cout << "Joint " << k+1 << " value: " << this->res_new.complete_solution[i].joint_matrix[j].data[k] << std::endl;
                                std::cout << "Joint Limits: " << this->joint_limits[count] << " " << this->joint_limits[count+1] << std::endl;
                            }
                        }
                    }
                    count += 2;
                }

                
            }
            iteratore = 0;
        }


        // std::cout << "IK solutions: " << std::endl;
        // std::cout << res << std::endl;

        res->complete_solution = this->res_new.complete_solution;
        res->solution = this->res_new.solution;
        res->move_q6 = this->res_new.move_q6;
        res->max_error = this->max_error;

        std::cout << "Max error: " << res->complete_solution[0].joint_matrix[0].data[0] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[0] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[1] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[2] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[3] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[4] << std::endl;
        // std::cout << res->complete_solution[1].joint_matrix[0].data[5] << std::endl;
        // std::cout << "Max error: " << res.max_error[0] << std::endl;
        // std::cout << "Max error: " << res.max_error[1] << std::endl;
        // std::cout << "Max error: " << res.max_error[2] << std::endl;
        // std::cout << "Max error: " << res.max_error[3] << std::endl;
        // std::cout << "Max error: " << res.max_error[4] << std::endl;
        // std::cout << "Max error: " << res.max_error[5] << std::endl;
        // std::cout << "Max error: " << res.max_error[6] << std::endl;
        // std::cout << "Max error: " << res.max_error[7] << std::endl;

        return true;
    }


    bool FIKServer::checkq6()
    {
        bool check = false;

        for (int i = 0; i < this->res_new.complete_solution.size(); i++)
        {
            for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
            {
                for (int k = 0; k < 6; k++)
                {
                    if(k==0 or k==3 or k==4 or k==5)
                    {
                        double diff;
                        if(i == 0)
                        {
                            diff = this->res_new.complete_solution[i].joint_matrix[j].data[k] - this->last_joints[k];
                        }
                        else
                        {
                            diff = this->res_new.complete_solution[i].joint_matrix[j].data[k] - this->res_new.complete_solution[i-1].joint_matrix[j].data[k];
                        }

                        if(diff > M_PI)
                        {
                            // nuovo intervallo [-pi, 3pi]
                            this->res_new.complete_solution[i].joint_matrix[j].data[k] -= 2*M_PI;
                            check = true;
                        }
                        else if(diff < -M_PI)
                        {
                            // nuovo intervallo [-3pi, pi]
                            this->res_new.complete_solution[i].joint_matrix[j].data[k] += 2*M_PI;
                            check = true;
                        }
                    }
                }
            }
        }

        if(check and this->ur_type != "TM900" and this->ur_type != "tm900")
        {   
            // cerco in tutti i punti e in tutte le configurazioni se l'ultimo giunto supera i limiti [-2pi, 2pi]
            for (int i = 0; i < this->res_new.complete_solution.size(); i++)
            {
                for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
                {
                    if (this->res_new.complete_solution[i].joint_matrix[j].data[5] > 2*M_PI)
                    {
                        // sottraggo 2pi a tutti i punti della configurazione attuale
                        for (int k = 0; k < this->res_new.complete_solution.size(); k++)
                        {
                            this->res_new.complete_solution[k].joint_matrix[j].data[5] -= 2*M_PI;
                        }
                        this->res_new.move_q6[j] = this->res_new.complete_solution[0].joint_matrix[j].data[5];
                    }
                    else if (this->res_new.complete_solution[i].joint_matrix[j].data[5] < -2*M_PI)
                    {
                        // sommo 2pi a tutti i punti della configurazione attuale
                        for (int k = 0; k < this->res_new.complete_solution.size(); k++)
                        {
                            this->res_new.complete_solution[k].joint_matrix[j].data[5] += 2*M_PI;
                        }
                        this->res_new.move_q6[j] = this->res_new.complete_solution[0].joint_matrix[j].data[5];
                    }
                }
            }
        }


        return check;
    }
}