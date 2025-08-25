


std::unique_ptr<RelativePosCostFunctor::CeresCostFunctionType>  RelativePosCostFunctor::create_cost_function_ptr(const Eigen::Vector3d& gnss_position_pre, const Eigen::Vector3d& gnss_position, double delta_t)  
{
        return std::unique_ptr<RelativePosCostFunctor::CeresCostFunctionType>(new RelativePosCostFunctor(gnss_position_pre, gnss_position, delta_t)).release();
}

RelativePosCostFunctor::RelativePosCostFunctor(Eigen::Vector3d gnss_position_pre, Eigen::Vector3d gnss_position, double delta_t)
: gnss_position_pre_(std::move(gnss_position_pre),  gnss_position(std::move(gnss_position)), delta_t_(delta_t))


