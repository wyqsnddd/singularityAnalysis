# ifndef JACOBIANGRADIENT_H
# define JACOBIANGRADIENT_H

// # include <pr2_hardware_interface/hardware_interface.h>
// # include <pr2_mechanism_model/robot.h>

# include <ros/ros.h>

# include <Eigen/Geometry>

class jacobianGradient{
 public:
  // necessary for Eigen library
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private: 
  enum{

    // Jacobian Dimention
    nReal = 7,
  };

  Eigen::Matrix<double, 6, nReal>  jacobian_;

  bool bodyJacobian_; 

  // gradient of each jacobian; 
  std::vector<Eigen::Matrix<double, 6, nReal> > jacobianGradientMatrixVector_;
  void updateJacobianGradient();

  // jacobian gradient operations: 
  Eigen::Matrix<double, 6, 1> lieBracket(const int screwIndex, const int jointIndex);

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> 
    skew_build( const Eigen::Matrix<double, 3, 1>  vector_in){
    //    Matrix_skew Matrix;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>  Matrix;
    
    Matrix << 0, -vector_in(2), vector_in(1),
      vector_in(2), 0, -vector_in(0),
      -vector_in(1), vector_in(0), 0;

    return Matrix;
    
  } 


  public: 

  jacobianGradient(const Eigen::Matrix<double, 6, nReal> &newJac, bool bodyJacobian  ){

    bodyJacobian_ = bodyJacobian;
    jacobian_.setZero();
    // There are nReal gradients 
    jacobianGradientMatrixVector_.resize(nReal);
    for(int i = 0; i<nReal;i++){
      jacobianGradientMatrixVector_[i].setZero();
    }
    
    // (1) write the jacobian
    updateJacobian(newJac);
    
    // (2) Initialize components

    // (3) give the first jacobian 
    //    updateJacobianGradient();

    //    std::cout<<"jacobianGradient is created"<<std::endl;
      
  }

  ~jacobianGradient();
  // (0)
  void updateJacobian(const Eigen::Matrix<double, 6, nReal> &newJac);
  // (1)
  Eigen::Matrix<double, 6, nReal> readJacobian()const {
    return jacobian_;
  }
  // (2)
  Eigen::Matrix<double, 6, nReal>  readJacobianGradient(int jointIndex)const {
    return jacobianGradientMatrixVector_[jointIndex];
  }
  std::vector<Eigen::Matrix<double, 6, nReal> > readJacobianGradient()const {
    return jacobianGradientMatrixVector_;
  }
};
# endif
