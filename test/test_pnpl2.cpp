#include <iostream>
#include <vector>
#include <math.h>
#include <random>

#include <eigen3/Eigen/Core>

#include "PnPL/pnpl.h"

#define PI 3.141592653589793

using namespace std;
using namespace pnpl;

class ProblemGenerator
{
public:
    /// camera_fov is in degree, camera_focal is in pixel.
    ProblemGenerator(int n_points, int n_lines, float camera_fov_w, float camera_fov_h, float camera_focal)
    : n_points_(n_points), n_lines_(n_lines), camera_focal_(camera_focal)
    {
        camera_fov_w_ = camera_fov_w / 180.0 * PI;
        camera_fov_h_ = camera_fov_h / 180.0 * PI;
        image_width_ = 2 * camera_focal * tan(0.5 * camera_fov_w_);
        image_height_ = 2 * camera_focal * tan(0.5 * camera_fov_h_);
        
        const float k = 1.4419737283123468;
        f = k * camera_focal_;
        cx = 0.5 * image_width_;
        cy = 0.5 * image_height_;
    }
    void Display()
    {
        cout<<"n_points = "<<n_points_<<endl;
        cout<<"n_lines = "<<n_lines_<<endl;
        cout<<"camera_fov_w = "<<camera_fov_w_<<endl;
        cout<<"camera_fov_h = "<<camera_fov_h_<<endl;
        cout<<"camera_focal = "<<camera_focal_<<endl;
        cout<<"image_width = "<<image_width_<<endl;
        cout<<"image_height = "<<image_height_<<endl;
        
        cout<<"intrinsics: "<<endl;
        cout<<"f = "<<f<<endl;
        cout<<"cx = "<<cx<<endl;
        cout<<"cy = "<<cy<<endl;
    }
    
    void generate(std::vector<Eigen::Vector3d>& pts3d, std::vector<Eigen::Vector2d>& pts2d, std::vector<Eigen::Vector6d>& lns3d, std::vector<Eigen::Vector4d>& lns2d, Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::Matrix3d& K)
    {
        cout<<"generating problem..."<<endl;
        float min_z = 0.2;
        float max_z = 25.0;
        float max_x = cx / f * max_z;
        float max_y = cy / f * max_z;
        cout<<"x range: ("<<-max_x<<","<<max_x<<")"<<endl;
        cout<<"y range: ("<<-max_y<<","<<max_y<<")"<<endl;
        cout<<"z range: ("<<min_z<<","<<max_z<<")"<<endl;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disx(-max_x,max_x);
        std::uniform_real_distribution<> disy(-max_y,max_y);
        std::uniform_real_distribution<> disz(min_z,max_z);
        std::uniform_real_distribution<> disr(0.0,PI);
        std::uniform_real_distribution<> dist(-10.0,10.0);
        
        double theta_x = disr(gen);
        double theta_y = disr(gen);
        double theta_z = disr(gen);
        R = Eigen::AngleAxisd(theta_x,Eigen::Vector3d::UnitX())*
            Eigen::AngleAxisd(theta_y,Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(theta_z,Eigen::Vector3d::UnitZ());
        
        t = Eigen::Vector3d(dist(gen),dist(gen),dist(gen));
        
        Eigen::Matrix3d Rinv = R.transpose();
        Eigen::Vector3d tinv = -Rinv * t;
        
        K << f, 0, cx,
             0, f, cy,
             0, 0, 1;
        
        int count = 0;
        while(count < n_points_)
        {
            Eigen::Vector3d X(disx(gen),disy(gen),disz(gen));
            Eigen::Vector2d x = Projection(X);
            if(isInView(x[0],x[1]))
            {
                Eigen::Vector3d Xw = Rinv * X + tinv;
                pts3d.push_back(Xw);
                pts2d.push_back(x);
                count++;
            }
        }
        
        count = 0;
        while(count < n_lines_)
        {
            Eigen::Vector3d Xp(disx(gen),disy(gen),disz(gen));
            Eigen::Vector3d Xe(disx(gen),disy(gen),disz(gen));
            
            Eigen::Vector2d xp = Projection(Xp);
            Eigen::Vector2d xe = Projection(Xe);
            
            if(isInView(xp[0],xp[1]) && isInView(xe[0],xe[1]))
            {
                Eigen::Vector3d Xpw = Rinv * Xp + tinv;
                Eigen::Vector3d Xew = Rinv * Xe + tinv;
                
                Eigen::Vector6d Xl;
                Xl<<Xpw,Xew;
                Eigen::Vector4d xl;
                xl<<xp,xe;
                
                lns3d.push_back(Xl);
                lns2d.push_back(xl);
                
                count++;
            }
        }
        
        double error = 0.0;
        for(int i=0;i<pts3d.size();i++)
        {
            Eigen::Vector3d Xc = R * pts3d[i] + t;
            Eigen::Vector2d x = Projection(Xc);
            error += (pts2d[i] - x).squaredNorm();
        }
        error /= pts3d.size();
        cout<<"reprojection error for points: "<<error<<endl;
        
        error = 0.0;
        for(int i=0;i<lns3d.size();i++)
        {
            Eigen::Vector3d Xsc = R * lns3d[i].head(3) + t;
            Eigen::Vector3d Xec = R * lns3d[i].tail(3) + t;
            error += (lns2d[i].head(2) - Projection(Xsc)).squaredNorm() + (lns2d[i].tail(2) - Projection(Xec)).squaredNorm();
        }
        error /= lns3d.size();
        cout<<"reprojection error for lines: "<<error<<endl;
        
        cout<<"done."<<endl;
    }
    
private:
    int n_points_;
    int n_lines_;
    float camera_fov_w_;
    float camera_fov_h_;
    float camera_focal_;
    float image_width_;
    float image_height_;
    
    float f, cx, cy;
    
    Eigen::Vector2d Projection(Eigen::Vector3d& X)
    {
        Eigen::Vector2d x;
        x[0] = f * X[0] / X[2] + cx;
        x[1] = f * X[1] / X[2] + cy;
        return x;
    }
    bool isInView(double u, double v)
    {
        if(u>=0 && u<image_width_ && v>=0 && v<image_height_)
        {
            return true;
        }
        return false;
    }
};


int main()
{
    ProblemGenerator gen(50,50,116.478966,86.972618,297.1548730884034);
    gen.Display();
    
    vector<Eigen::Vector3d> pts3d;
    vector<Eigen::Vector2d> pts2d;
    vector<Eigen::Vector6d> lns3d;
    vector<Eigen::Vector4d> lns2d;
    Eigen::Matrix3d R,K;
    Eigen::Vector3d t;
    
    gen.generate(pts3d,pts2d,lns3d,lns2d,R,t,K);
    
    cout<<"groundtruth:"<<endl;
    cout<<"R  = "<<endl<<R<<endl;
    cout<<"t = "<<t.transpose()<<endl;
    cout<<"pts3d.size = "<<pts3d.size()<<endl;
    cout<<"lns3d.size = "<<lns3d.size()<<endl;
    
    cout<<"solving (R,t) by PnPL ..."<<endl;
    Eigen::Matrix3d Ri;
    Eigen::Vector3d ti;
    PnPL(pts3d,pts2d,lns3d,lns2d,K,Ri,ti);
    cout<<"results:"<<endl;
    cout<<"R = "<<endl<<R<<endl;
    cout<<"t = "<<t.transpose()<<endl;
    
    return 0;
}
