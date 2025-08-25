// IMG_0997



///  Cost functor that implents a relative position constraint by comparing the measurement 
/// between wheel odometry and gps measurement 

class RelativePosCostFunctor
{
    public:
        /// @brief  The type of cost function that wraps the cost functor.
        /// - 2 residual dimensions
        /// - 1x1 scalar representing the scalar of the wheel speed
        using CeresCostFunctionType = ceres:: AutoDiffCostFunction<
        RelativePosCostFunctor,
        relative_position_residual_dimension, // 2
        wheel_scalar_dimension,  // 1
        gps_heading_offset_dimension>; // 1

     ///  static factory method to create and return a CostFunction pointer.  
     static std::unique_ptr<CeresCostFunctionType> create_cost_function_ptr( const double observation);
     


     /// Computes residuals for the cost functor. The residual is defined as the difference between 
     /// the movement of the gps antenna in the ENU-frame, but we only care he 2D result

     /// @tparam  T the scalar type (a double or ceres::Jet for auto diff).
     /// @param[in] wheel_speed_scalar_ptr Pointer to the wheel speed scalar
     /// @param[in] gps_heading_offset_ptr Pointer to the gps heading offset
     /// @param[out]  residual_ptr Pointer to the output residual   
     /// @return     True on success or false if otherwise.
      
    template <typename T>
    bool operator()(
        const T* const wheel_speed_scalar_ptr, 
        const T* const gps_heading_offset_ptr,
        T* residual_ptr) const;

  private:
        /// Construct a relative position cost functor.
        /// @param[in] 

        RelativePosCostFunctor(Eigen::Vector3d gnss_position_pre, Eigen::Vector3d gnss_position, double delta_t);

        
        /// Previous gnss position measurement
        Eigen::Vector3d gnss_position_pre_; 

        /// Current gnss postion measurement
        Eigen::Vector3d gnss_position_;      
        
        /// Time difference between the previous measurement and current one
        double delta_t_;  
};

#include "constraint_generator.inl"  // inside the file, put the definition of operator


