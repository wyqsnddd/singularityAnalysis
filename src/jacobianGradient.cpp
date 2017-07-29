# include <jacobianGradient/jacobianGradient.h>

void jacobianGradient::updateJacobian(const Eigen::Matrix<double, 6, nReal> &newJac){

  for(unsigned int i=0; i<6; i++)
    for(unsigned int j=0; j<nReal; j++)
      jacobian_(i, j) = newJac(i, j);


  //  std::cout<<"jacobianGradient is about to calculate the gradient"<<std::endl;
  // update the gradient 
  updateJacobianGradient();
  //  std::cout<<"jacobianGradient is done"<<std::endl;

}

// Eigen::Matrix<double, 6, 1> jacobianGradient::lieBracket(const int screwIndex, const int jointIndex){

//   int j = jointIndex;
//   int i = screwIndex;
Eigen::Matrix<double, 6, 1> jacobianGradient::lieBracket(const int j, const int i){

  Eigen::Matrix<double, 6, 1>  result;

  if(i>=j){
  // wj x wi

  result.block<3,1>(3,0) = 
    skew_build(jacobian_.block<3,1>(3,j))
    *jacobian_.block<3,1>(3,i);

  // wj x vi - vj x wi
  result.block<3,1>(0,0) = 
    skew_build(jacobian_.block<3,1>(3,j))*jacobian_.block<3,1>(0,i)
    - 
    skew_build(jacobian_.block<3,1>(0,j))*jacobian_.block<3,1>(3,i);

  }else{
    result.setZero();
  }
  return result;  

}
void jacobianGradient::updateJacobianGradient(){

  

  // loop over the joints 
  for(int j = 0; j<nReal;j++){
    // loop over the columns:  

	
    for(int i = 0; i<nReal; i++){
      // if(bodyJacobian_)
      // 	jacobianGradientMatrixVector_[j].block<6,1>(0,i) = lieBracket(j,i);
      // else
      // 	jacobianGradientMatrixVector_[j].block<6,1>(0,i) = lieBracket(i,j);
      if(bodyJacobian_)
	jacobianGradientMatrixVector_[j].block<6,1>(0,i) = lieBracket(i,j);
      else
	jacobianGradientMatrixVector_[j].block<6,1>(0,i) = lieBracket(j,i);
    }
    //    std::cout<<"one jacobian is done "<<std::endl;
  }
}
