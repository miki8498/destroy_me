#include "rclcpp/rclcpp.hpp"
#include "Spline.hpp"


namespace ur_c_spline {

    Spline::Spline(int rate): Node("interpolator")
    {

        this->initialiseRos();
        
    }


    Spline::~Spline()
    {

    }


    void Spline::initialiseRos()
    {   
        this->interpolate_service = create_service<message::srv::Interpolate>("interpolate", std::bind(&Spline::interpolate_serviceCB, this, std::placeholders::_1, std::placeholders::_2));

        this->joint_interpolate_service = create_service<message::srv::JointInterpolate>("joint_interpolate", std::bind(&Spline::joint_interpolate_serviceCB, this, std::placeholders::_1, std::placeholders::_2));

        this->interpolate_service_linear = create_service<message::srv::Interpolate>("interpolate_linear", std::bind(&Spline::interpolate_service_linearCB, this, std::placeholders::_1, std::placeholders::_2));

        std::cout<<"Spline Service Initialised"<<std::endl;
    }


    void Spline::initialiseFilters(float period)
    {

        //create FIR filter vector
        this->filter.resize(3);
        for (int i=0;i<3;i++)
        {
            this->filter[i] = new fir_filter::FIRFilter(1,1.0/this->rate_spline);
        }

        //create input vector
        Vector6d ref_pose;
        ref_pose[0] = this->start_pose.position.x;
        ref_pose[1] = this->start_pose.position.y;
        ref_pose[2] = this->start_pose.position.z;
        ref_pose[3] = 0;
        ref_pose[4] = 0;
        ref_pose[5] = 0;
        //initialise filters
        for (int i=0;i<3;i++)
        {
            this->filter[i]->setFilterParameters(period,1.0/this->rate_spline);
            this->filter[i]->initialiseFilter(ref_pose);
        }   

        //evaluate convergence steps
        this->slerp_steps = 3*this->filter[0]->getFilterCellNumber();     
    }

    void Spline::initialiseFiltersLinear(float period)
    {

        //create FIR filter vector
        this->filter.resize(3);
        for (int i=0;i<3;i++)
        {
            this->filter[i] = new fir_filter::FIRFilter(1,1.0/this->rate_spline);
        }

        //create input vector
        Vector6d ref_pose;
        ref_pose[0] = 0;
        ref_pose[1] = 0;
        ref_pose[2] = 0;
        ref_pose[3] = 0;
        ref_pose[4] = 0;
        ref_pose[5] = 0;
        //initialise filters
        for (int i=0;i<3;i++)
        {
            this->filter[i]->setFilterParameters(period,1.0/this->rate_spline);
            this->filter[i]->initialiseFilter(ref_pose);
        }   

        //evaluate convergence steps
        this->slerp_steps = 3*this->filter[0]->getFilterCellNumber();     
    }


    void Spline::initialiseFiltersJoints(float period)
    {
        //create input vector
        Vector6d ref_pose;

        //create FIR filter vector
        this->filter.resize(3);
        for (int i=0;i<3;i++)
        {
            this->filter[i] = new fir_filter::FIRFilter(1,1.0/this->rate_spline);
        }

        for (int i=0;i<6;i++)
        {
            ref_pose[i] = this->start_joints.data[i];
        }

        //initialise filters
        for (int i=0;i<3;i++)
        {
            this->filter[i]->setFilterParameters(period,1.0/this->rate_spline);
            this->filter[i]->initialiseFilter(ref_pose);
        }   

        //evaluate convergence steps
        this->slerp_steps = 3*this->filter[0]->getFilterCellNumber();     

    }


    bool Spline::interpolate_serviceCB(const std::shared_ptr<message::srv::Interpolate::Request> req, std::shared_ptr<message::srv::Interpolate::Response> res)
    {
        this->rate_spline = req->rate;
        this->start_pose = req->start_pose;
        this->goal_pose = req->goal_pose;

        float distance = std::sqrt(std::pow(this->goal_pose.position.x - this->start_pose.position.x,2) + std::pow(this->goal_pose.position.y - this->start_pose.position.y,2) + std::pow(this->goal_pose.position.z - this->start_pose.position.z,2));
        // da m a cm
        distance = distance*100;
        this->period = distance/req->vel_cm_s;


        this->initialiseFilters(this->period);

        //define temp variables
        Vector6d input,output;

        geometry_msgs::msg::Pose temp_pose;
        int count;

        //prepare variables for evaluation
        input <<this->goal_pose.position.x,this->goal_pose.position.y,this->goal_pose.position.z,0,0,0;

        Eigen::Quaterniond qa(this->start_pose.orientation.w,this->start_pose.orientation.x,this->start_pose.orientation.y,this->start_pose.orientation.z);
        Eigen::Quaterniond qb(this->goal_pose.orientation.w,this->goal_pose.orientation.x,this->goal_pose.orientation.y,this->goal_pose.orientation.z);

        qa.normalize();
        qb.normalize();

        float dot = qa.dot(qb);
        if (dot < 0.0f)
            qb = quaternion_negation(qb);


        // loop to evaluate the trajectory
        for (int i = 0;i<this->slerp_steps; ++i)
        {
            //evaluate position with spline filter
            this->filter[0]->addNewInput(input);
            count = 0;

            for (int j = 1; j < 3; ++j)
            {
                count = j;
                this->filter[j]->addNewInput(this->filter[j-1]->evaluateFilterOutput());
            }

            output = this->filter[count]->evaluateFilterOutput();

            temp_pose.position.x = output[0];
            temp_pose.position.y = output[1];
            temp_pose.position.z = output[2];

            //evaluate orientation with slerp
            Eigen::Quaterniond qres = qa.slerp(i / (float)(this->slerp_steps-1.0), qb);
            temp_pose.orientation.w = qres.w();
            temp_pose.orientation.x = qres.x();
            temp_pose.orientation.y = qres.y();
            temp_pose.orientation.z = qres.z();
            
            this->trajectory.push_back(temp_pose);           

        }

        res->trajectory = this->trajectory;
        res->success = true;

        //clear trajectory
        this->trajectory.clear();
        //clear filters
        for (int i=0;i<3;i++)
        {
            delete this->filter[i];
        }

        return res->success;
    }

    bool Spline::interpolate_service_linearCB(const std::shared_ptr<message::srv::Interpolate::Request> req, std::shared_ptr<message::srv::Interpolate::Response> res)
    {
        this->rate_spline = req->rate;
        this->start_pose = req->start_pose;
        this->goal_pose = req->goal_pose;

        float distance = std::sqrt(std::pow(this->goal_pose.position.x - this->start_pose.position.x,2) + std::pow(this->goal_pose.position.y - this->start_pose.position.y,2) + std::pow(this->goal_pose.position.z - this->start_pose.position.z,2));
        // da m a cm
        distance = distance*100;
        this->period = req->vel_cm_s/3.0;

        this->initialiseFiltersLinear(this->period);

        //define temp variables
        Vector6d input,output;

        geometry_msgs::msg::Pose temp_pose;
        int count;

        //prepare variables for evaluation
        input <<1,1,1,0,0,0;

        Eigen::Quaterniond qa(this->start_pose.orientation.w,this->start_pose.orientation.x,this->start_pose.orientation.y,this->start_pose.orientation.z);
        Eigen::Quaterniond qb(this->goal_pose.orientation.w,this->goal_pose.orientation.x,this->goal_pose.orientation.y,this->goal_pose.orientation.z);

        qa.normalize();
        qb.normalize();

        float dot = qa.dot(qb);
        if (dot < 0.0f)
            qb = quaternion_negation(qb);


        // loop to evaluate the trajectory
        for (int i = 0;i<this->slerp_steps; ++i)
        {
            //evaluate position with spline filter
            this->filter[0]->addNewInput(input);
            count = 0;

            for (int j = 1; j < 3; ++j)
            {
                count = j;
                this->filter[j]->addNewInput(this->filter[j-1]->evaluateFilterOutput());
            }

            output = this->filter[count]->evaluateFilterOutput();

            temp_pose.position.x = this->start_pose.position.x + output[0]*(this->goal_pose.position.x - this->start_pose.position.x);
            temp_pose.position.y = this->start_pose.position.y + output[1]*(this->goal_pose.position.y - this->start_pose.position.y);
            temp_pose.position.z = this->start_pose.position.z + output[2]*(this->goal_pose.position.z - this->start_pose.position.z);

            //evaluate orientation with slerp
            Eigen::Quaterniond qres = qa.slerp(i / (float)(this->slerp_steps-1.0), qb);
            temp_pose.orientation.w = qres.w();
            temp_pose.orientation.x = qres.x();
            temp_pose.orientation.y = qres.y();
            temp_pose.orientation.z = qres.z();
            
            this->trajectory.push_back(temp_pose);           

        }

        res->trajectory = this->trajectory;
        res->success = true;

        //clear trajectory
        this->trajectory.clear();
        //clear filters
        for (int i=0;i<3;i++)
        {
            delete this->filter[i];
        }

        return res->success;
    }


    bool Spline::joint_interpolate_serviceCB(const std::shared_ptr<message::srv::JointInterpolate::Request> req, std::shared_ptr<message::srv::JointInterpolate::Response> res)
    {
        Vector6d input,output;

        int count;
        std_msgs::msg::Float64MultiArray temp_joints;
        temp_joints.data.resize(6);

        this->rate_spline = req->rate;
        this->period = req->period;
        this->start_joints = req->start_joints;

        this->initialiseFiltersJoints(this->period);

        input <<req->goal_joints.data[0],req->goal_joints.data[1],req->goal_joints.data[2],req->goal_joints.data[3],req->goal_joints.data[4],req->goal_joints.data[5];

        // confronto se req.start_joints e' uguale a req.goal_joints e mi salvo gli indici dei giunti uguali
        std::vector<int> equal_joints;
        for (int i = 0; i < 6; i++)
        {
            if (req->start_joints.data[i] == req->goal_joints.data[i])
            {
                equal_joints.push_back(i);
            }
        }

        // loop to evaluate the trajectory
        for (int i = 0;i<this->slerp_steps; ++i)
        {
            //evaluate position with spline filter
            this->filter[0]->addNewInput(input);
            count = 0;

            for (int j = 1; j < 3; ++j)
            {
                count = j;
                this->filter[j]->addNewInput(this->filter[j-1]->evaluateFilterOutput());
            }

            output = this->filter[count]->evaluateFilterOutput();

            for (int j = 0; j < 6; j++)
            {
                //confronto j con gli indici dei giunti uguali, se sono uguali allora metto il valore di start_joints altrimenti metto il valore di output
                if (std::find(equal_joints.begin(), equal_joints.end(), j) != equal_joints.end())
                {
                    temp_joints.data[j] = req->start_joints.data[j];
                }
                else
                {
                    temp_joints.data[j] = output[j];
                }

            }

            this->joints_trajectory.push_back(temp_joints);         

        }

        res->joints_trajectory = this->joints_trajectory;
        res->success = true;

        //clear trajectory
        this->joints_trajectory.clear();
        //clear filters
        for (int i=0;i<3;i++)
        {
            delete this->filter[i];
        }

        return res->success;
    }


    Eigen::Quaterniond Spline::quaternion_negation(Eigen::Quaterniond v) 
    {
        Eigen::Quaterniond res;
        res.w() = -v.w();
        res.x() = -v.x();
        res.y() = -v.y();
        res.z() = -v.z();

        return res;
    }

}