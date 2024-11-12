#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP


struct RobotModel {
    public:
    struct Link {
    float length;
    float x;
    float y;
    float z; 
    }; 
    RobotModel::Link links[6] = {
        {2.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},          
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0}
    };

};

#endif