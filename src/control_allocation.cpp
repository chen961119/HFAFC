#include "control_allocation.h"
#include "control_state.h"
#include "sensor_processing.h"

using namespace Eigen;

static MatrixXf M_pinv(18, 18);
static MatrixXf H(35, 20);   // 使用float单精度
static MatrixXf Mp(10, 18);  // 共提取10行（3+3+1+1+1+1）
static MatrixXf Mpi(10, 10); // 共提取10行（3+3+1+1+1+1）
static MatrixXf M(18, 18);
static MatrixXf Bplusmini(4, 20);
static MatrixXf Bplusminismall(4, 10);
static MatrixXf Bplusfull(6, 20);
static MatrixXf Bplusfullsmall(6, 10);
MatrixXf Bplusminismall_pinv(10, 4);
MatrixXf Bplusfullsmall_pinv(10, 6);
static Matrix3f Eab, Eac, Ebd, Ece;
static Matrix3f Ia, Ib, Ic, Id, Ie;
VectorXf dw_config(4);  // 四个变形通道
VectorXf dw_att(6);     // 6个整体运动通道
VectorXf de_att(10);    // 只考虑所有的副翼
VectorXf de_config(10); // 只考虑所有的副翼
static VectorXf Qx(10);        // 广义力
static VectorXf dw_rev(10);    // 广义加速度

// 常量定义（与MATLAB完全一致）
static float Mass = 0.8;
static float Mbss = 0.8;
static float Mcss = 0.8;
static float Mdss = 0.8;
static float Mess = 0.8;

// 惯性张量（严格对应MATLAB）
static float iaxx = 0.05, iaxy = 0, iaxz = 0.0, iayy = 0.08, iayz = 0, iazz = 0.13;
static float ibxx = 0.05, ibxy = 0, ibxz = 0.0, ibyy = 0.08, ibyz = 0, ibzz = 0.13;
static float icxx = 0.05, icxy = 0, icxz = 0.0, icyy = 0.08, icyz = 0, iczz = 0.13;
static float idxx = 0.05, idxy = 0, idxz = 0, idyy = 0.08, idyz = 0, idzz = 0.13;
static float iexx = 0.05, iexy = 0, iexz = 0, ieyy = 0.08, ieyz = 0, iezz = 0.13;

// 几何参数
static float span = 1.5; // 需要设置实际值
static float y = 0.5 * span;
static float c = 0.21;
static float S = 0.32;

// 位置向量（严格对应MATLAB）
static Vector3f Rab(0, -y, 0);
static Vector3f Rbm(0, -y, 0);
static Vector3f Rac(0, y, 0);
static Vector3f Rcm(0, y, 0);
static Vector3f Rbd(0, -y, 0);
static Vector3f Rdm(0, -y, 0);
static Vector3f Rce(0, y, 0);
static Vector3f Rem(0, y, 0);

MatrixXf computeBeMatrix(float y, float phiab, float phiac, float phibd, float phice);
void extractMpi();

//矩阵运算
void initializeInertiaMatrices(Matrix3f& Ia, Matrix3f& Ib, Matrix3f& Ic, Matrix3f& Id, Matrix3f& Ie);
void initializeRotationMatrices(Matrix3f& Eab, Matrix3f& Eac, Matrix3f& Ebd, Matrix3f& Ece);
void initializeH(); 
void getpinvBplusmini();

// 叉乘矩阵函数（严格对应MATLAB的fan函数）
Matrix3f fan(const Vector3f& v) {
    Matrix3f m;
    m << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return m;
}

// 初始化惯性张量（严格对应MATLAB）
void initializeInertiaMatrices(Matrix3f& Ia, Matrix3f& Ib, Matrix3f& Ic, Matrix3f& Id, Matrix3f& Ie) {
    Ia << iaxx, -iaxy, -iaxz,
         -iaxy,  iayy, -iayz,
         -iaxz, -iayz,  iazz;
    
    Ib << ibxx, -ibxy, -ibxz,
         -ibxy,  ibyy, -ibyz,
         -ibxz, -ibyz,  ibzz;
    
    Ic << icxx, -icxy, -icxz,
         -icxy,  icyy, -icyz,
         -icxz, -icyz,  iczz;
    
    Id << idxx, -idxy, -idxz,
         -idxy,  idyy, -idyz,
         -idxz, -idyz,  idzz;
    
    Ie << iexx, -iexy, -iexz,
         -iexy,  ieyy, -ieyz,
         -iexz, -ieyz,  iezz;
}

// 初始化旋转矩阵（严格对应MATLAB）
void initializeRotationMatrices(Matrix3f& Eab, Matrix3f& Eac, Matrix3f& Ebd, Matrix3f& Ece) {
    float phiabrad=relativeAngle_ready*0.01745;
    float phiacrad=phiac*0.01745;
    float phibdrad=phibd*0.01745;
    float phicerad=phice*0.01745;
    Eab << 1, 0, 0,
           0, cos(phiabrad), sin(phiabrad),
           0, -sin(phiabrad), cos(phiabrad);
    
    Eac << 1, 0, 0,
           0, cos(phiacrad), sin(phiacrad),
           0, -sin(phiacrad), cos(phiacrad);
    
    Ebd << 1, 0, 0,
           0, cos(phibdrad), sin(phibdrad),
           0, -sin(phibdrad), cos(phibdrad);
    
    Ece << 1, 0, 0,
           0, cos(phicerad), sin(phicerad),
           0, -sin(phicerad), cos(phicerad);
}

// 计算所有C矩阵（严格逐项对应MATLAB代码）
void calculateCMatrices(Matrix3f& C11, Matrix3f& C12, Matrix3f& C13, Matrix3f& C14, 
                       Matrix3f& C15, Matrix3f& C16, Matrix3f& C21, Matrix3f& C22,
                       Matrix3f& C23, Matrix3f& C24, Matrix3f& C25, Matrix3f& C26,
                       Matrix3f& C31, Matrix3f& C32, Matrix3f& C33, Matrix3f& C34,
                       Matrix3f& C35, Matrix3f& C36, Matrix3f& C41, Matrix3f& C42,
                       Matrix3f& C43, Matrix3f& C44, Matrix3f& C45, Matrix3f& C46,
                       Matrix3f& C51, Matrix3f& C52, Matrix3f& C53, Matrix3f& C54,
                       Matrix3f& C55, Matrix3f& C56, Matrix3f& C61, Matrix3f& C62,
                       Matrix3f& C63, Matrix3f& C64, Matrix3f& C65, Matrix3f& C66) {
    
    // 初始化矩阵
    //Matrix3f Ia, Ib, Ic, Id, Ie;
    //initializeInertiaMatrices(Ia, Ib, Ic, Id, Ie);
    
    //Matrix3f Eab, Eac, Ebd, Ece;
    //initializeRotationMatrices(Eab, Eac, Ebd, Ece);

    // 严格按MATLAB顺序计算每个C矩阵 ----------------------------
    
    // C11计算（完全一致）
    C11 = (Mass + Mbss + Mcss + Mdss + Mess) * Matrix3f::Identity();
    
    // C12计算（完全一致）
    C12 = Mbss*(fan(Rab).transpose() + Eab.transpose()*fan(Rbm).transpose()*Eab)
        + Mcss*(fan(Rac).transpose() + Eac.transpose()*fan(Rcm).transpose()*Eac)
        + Mdss*Eab.transpose()*(Eab*fan(Rab).transpose()*Eab.transpose() 
           + fan(Rbm).transpose() + fan(Rbd).transpose() 
           + Ebd.transpose()*fan(Rdm).transpose()*Ebd)*Eab
        + Mess*Eac.transpose()*(Eac*fan(Rac).transpose()*Eac.transpose() 
           + fan(Rcm).transpose() + fan(Rce).transpose() 
           + Ece.transpose()*fan(Rem).transpose()*Ece)*Eac;
    
    // C13计算（完全一致）
    C13 = Mbss*Eab.transpose()*fan(Rbm).transpose()
        + Mdss*Eab.transpose()*(fan(Rbm).transpose() + fan(Rbd).transpose() 
           + Ebd.transpose()*fan(Rdm).transpose()*Ebd);
    
    // C14计算（完全一致）
    C14 = Mcss*Eac.transpose()*fan(Rcm).transpose()
        + Mess*Eac.transpose()*(fan(Rcm).transpose() + fan(Rce).transpose() 
           + Ece.transpose()*fan(Rem).transpose()*Ece);
    
    // C15计算（完全一致）
    C15 = Mdss*Eab.transpose()*Ebd.transpose()*fan(Rdm).transpose();
    
    // C16计算（完全一致）
    C16 = Mess*Eac.transpose()*Ece.transpose()*fan(Rem).transpose();
    
    // C21计算（完全一致）
    C21 = Mbss*(fan(Rab) + Eab.transpose()*fan(Rbm)*Eab)
        + Mcss*(fan(Rac) + Eac.transpose()*fan(Rcm)*Eac)
        + Mdss*Eab.transpose()*(Eab*fan(Rab)*Eab.transpose() 
           + fan(Rbm) + fan(Rbd) + Ebd.transpose()*fan(Rdm)*Ebd)*Eab
        + Mess*Eac.transpose()*(Eac*fan(Rac)*Eac.transpose() 
           + fan(Rcm) + fan(Rce) + Ece.transpose()*fan(Rem)*Ece)*Eac;
    
    // C22计算（完全一致）
    C22 = Ia + Eab.transpose()*Ib*Eab + Eac.transpose()*Ic*Eac 
        + Eab.transpose()*Ebd.transpose()*Id*Ebd*Eab 
        + Eac.transpose()*Ece.transpose()*Ie*Ece*Eac
        + Mbss*(fan(Rab) + Eab.transpose()*fan(Rbm)*Eab)
          *(fan(Rab).transpose() + Eab.transpose()*fan(Rbm).transpose()*Eab)
        + Mcss*(fan(Rac) + Eac.transpose()*fan(Rcm)*Eac)
          *(fan(Rac).transpose() + Eac.transpose()*fan(Rcm).transpose()*Eac)
        + Mdss*Eab.transpose()*(Eab*fan(Rab)*Eab.transpose() + fan(Rbm) + fan(Rbd) 
          + Ebd.transpose()*fan(Rdm)*Ebd)
          *(Eab*fan(Rab).transpose()*Eab.transpose() + fan(Rbm).transpose() 
          + fan(Rbd).transpose() + Ebd.transpose()*fan(Rdm).transpose()*Ebd)*Eab
        + Mess*Eac.transpose()*(Eac*fan(Rac)*Eac.transpose() + fan(Rcm) + fan(Rce) 
          + Ece.transpose()*fan(Rem)*Ece)
          *(Eac*fan(Rac).transpose()*Eac.transpose() + fan(Rcm).transpose() 
          + fan(Rce).transpose() + Ece.transpose()*fan(Rem).transpose()*Ece)*Eac;
    
    // C23计算（完全一致）
    C23 = Mbss*(fan(Rab) + Eab.transpose()*fan(Rbm)*Eab)*Eab.transpose()*fan(Rbm).transpose()
        + Eab.transpose()*Ib + Eab.transpose()*Ebd.transpose()*Id*Ebd
        + Mdss*Eab.transpose()*(Eab*fan(Rab)*Eab.transpose() + fan(Rbm) + fan(Rbd) 
          + Ebd.transpose()*fan(Rdm)*Ebd)
          *(Ebd.transpose()*fan(Rdm).transpose()*Ebd + fan(Rbm).transpose() 
          + fan(Rbd).transpose());
    
    // C24计算（完全一致）
    C24 = Mcss*(fan(Rac) + Eac.transpose()*fan(Rcm)*Eac)*Eac.transpose()*fan(Rcm).transpose()
        + Eac.transpose()*Ic + Eac.transpose()*Ece.transpose()*Ie*Ece
        + Mess*Eac.transpose()*(Eac*fan(Rac)*Eac.transpose() + fan(Rcm) + fan(Rce) 
          + Ece.transpose()*fan(Rem)*Ece)
          *(Ece.transpose()*fan(Rem).transpose()*Ece + fan(Rcm).transpose() 
          + fan(Rce).transpose());
    
    // C25计算（完全一致）
    C25 = Mdss*Eab.transpose()*(Eab*fan(Rab)*Eab.transpose() + fan(Rbm) + fan(Rbd) 
          + Ebd.transpose()*fan(Rdm)*Ebd)*Ebd.transpose()*fan(Rdm).transpose()
        + Eab.transpose()*Ebd.transpose()*Id;
    
    // C26计算（完全一致）
    C26 = Mess*Eac.transpose()*(Eac*fan(Rac)*Eac.transpose() + fan(Rcm) + fan(Rce) 
          + Ece.transpose()*fan(Rem)*Ece)*Ece.transpose()*fan(Rem).transpose()
        + Eac.transpose()*Ece.transpose()*Ie;
    
    // C31计算（完全一致）
    C31 = Mbss*fan(Rbm)*Eab
        + Mdss*(fan(Rbm) + fan(Rbd) + Ebd.transpose()*fan(Rdm)*Ebd)*Eab;
    
    // C32计算（完全一致）
    C32 = Mbss*fan(Rbm)*Eab*(fan(Rab).transpose() + Eab.transpose()*fan(Rbm).transpose()*Eab)
        + Ib*Eab + Ebd.transpose()*Id*Ebd*Eab
        + Mdss*(fan(Rbm) + fan(Rbd) + Ebd.transpose()*fan(Rdm)*Ebd)
          *(Eab*fan(Rab).transpose()*Eab.transpose() + fan(Rbm).transpose() 
          + fan(Rbd).transpose() + Ebd.transpose()*fan(Rdm).transpose()*Ebd)*Eab;
    
    // C33计算（完全一致）
    // 修正后的 C33 计算（严格保持MATLAB公式结构）
    C33 = Mbss*fan(Rbm)*fan(Rbm).transpose() + Ib + Ebd.transpose()*Id*Ebd
     + Mdss*(fan(Rbm) + fan(Rbd) + Ebd.transpose()*fan(Rdm)*Ebd)
      *(fan(Rbm).transpose() + fan(Rbd).transpose() + Ebd.transpose()*fan(Rdm).transpose()*Ebd);

    // C34计算（完全一致）
    C34 = Matrix3f::Zero();
    
    // C35计算（完全一致）
    C35 = Ebd.transpose()*Id
        + Mdss*(fan(Rbm) + fan(Rbd) + Ebd.transpose()*fan(Rdm)*Ebd)*Ebd.transpose()*fan(Rdm).transpose();
    
    // C36计算（完全一致）
    C36 = Matrix3f::Zero();
    
    // C41计算（完全一致）
    C41 = Mcss*fan(Rcm)*Eac
        + Mess*(fan(Rcm) + fan(Rce) + Ece.transpose()*fan(Rem)*Ece)*Eac;
    
    // C42计算（完全一致）
    C42 = Mcss*fan(Rcm)*Eac*(fan(Rac).transpose() + Eac.transpose()*fan(Rcm).transpose()*Eac)
        + Ic*Eac + Ece.transpose()*Ie*Ece*Eac
        + Mess*(fan(Rcm) + fan(Rce) + Ece.transpose()*fan(Rem)*Ece)
          *(Eac*fan(Rac).transpose()*Eac.transpose() + fan(Rcm).transpose() 
          + fan(Rce).transpose() + Ece.transpose()*fan(Rem).transpose()*Ece)*Eac;
    
    // C43计算（完全一致）
    C43 = Matrix3f::Zero();
    
    // C44计算（完全一致）
    C44 = Ic + Mcss*fan(Rcm)*fan(Rcm).transpose() + Ece.transpose()*Ie*Ece
        + Mess*(fan(Rcm)*Ece.transpose() + fan(Rce)*Ece.transpose() 
          + Ece.transpose()*fan(Rem))*(Ece*fan(Rcm).transpose() + Ece*fan(Rce).transpose() 
          + fan(Rem).transpose()*Ece);
    

    
    // C45计算（完全一致）
    C45 = Matrix3f::Zero();
    
    // C46计算（完全一致）
    C46 = Mess*(fan(Rcm) + fan(Rce) + Ece.transpose()*fan(Rem)*Ece)*Ece.transpose()*fan(Rem).transpose()
        + Ece.transpose()*Ie;
    
    // C51计算（完全一致）
    C51 = Mdss*fan(Rdm)*Ebd*Eab;
    
    // C52计算（完全一致）
    C52 = Mdss*fan(Rdm)*Ebd*(Eab*fan(Rab).transpose()*Eab.transpose() 
          + fan(Rbm).transpose() + fan(Rbd).transpose() 
          + Ebd.transpose()*fan(Rdm).transpose()*Ebd)*Eab
        + Id*Ebd*Eab;
    
    // C53计算（完全一致）
    C53 = Mdss*fan(Rdm)*(Ebd*fan(Rbm).transpose()*Ebd.transpose() 
          + Ebd*fan(Rbd).transpose()*Ebd.transpose() 
          + fan(Rdm).transpose())*Ebd
        + Id*Ebd;
    
    // C54计算（完全一致）
    C54 = Matrix3f::Zero();
    
    // C55计算（完全一致）
    C55 = Mdss*fan(Rdm)*fan(Rdm).transpose() + Id;
    
    // C56计算（完全一致）
    C56 = Matrix3f::Zero();
    
    // C61计算（完全一致）
    C61 = Mess*fan(Rem)*Ece*Eac;
    
    // C62计算（完全一致）
    C62 = Mess*fan(Rem)*Ece*(Eac*fan(Rac).transpose()*Eac.transpose() 
          + fan(Rcm).transpose() + fan(Rce).transpose() 
          + Ece.transpose()*fan(Rem).transpose()*Ece)*Eac
        + Ie*Ece*Eac;
    
    // C63计算（完全一致）
    C63 = Matrix3f::Zero();
    
    // C64计算（完全一致）
    C64 = (Mess*fan(Rem)*(Ece*fan(Rcm).transpose()*Ece.transpose() 
          + Ece*fan(Rce).transpose()*Ece.transpose() 
          + fan(Rem).transpose()) + Ie)*Ece;
    
    // C65计算（完全一致）
    C65 = Matrix3f::Zero();
    
    // C66计算（完全一致）
    C66 = Mess*fan(Rem)*fan(Rem).transpose() + Ie;
}

// 组装完整质量矩阵（严格对应MATLAB）
MatrixXf assembleMassMatrix() {
    Matrix3f C11, C12, C13, C14, C15, C16;
    Matrix3f C21, C22, C23, C24, C25, C26;
    Matrix3f C31, C32, C33, C34, C35, C36;
    Matrix3f C41, C42, C43, C44, C45, C46;
    Matrix3f C51, C52, C53, C54, C55, C56;
    Matrix3f C61, C62, C63, C64, C65, C66;
    
    calculateCMatrices(C11, C12, C13, C14, C15, C16,
                      C21, C22, C23, C24, C25, C26,
                      C31, C32, C33, C34, C35, C36,
                      C41, C42, C43, C44, C45, C46,
                      C51, C52, C53, C54, C55, C56,
                      C61, C62, C63, C64, C65, C66);
    
    // 组装18x18矩阵（完全对应MATLAB结构）

    M << C11, C12, C13, C14, C15, C16,
         C21, C22, C23, C24, C25, C26,
         C31, C32, C33, C34, C35, C36,
         C41, C42, C43, C44, C45, C46,
         C51, C52, C53, C54, C55, C56,
         C61, C62, C63, C64, C65, C66;
    
    /*
    // 设置特定列为0（完全对应MATLAB）
    for (int i : {7, 8, 10, 11, 13, 14, 16, 17}) {
        M.col(i).setZero();
        M.row(i).setZero();
    }
    */
    
    return M;
}

void getQx()
{
  Eigen::Vector3f FAero(0, 0, -Mass * 9.8);
  Eigen::Vector3f FBero(0, 0, -Mbss * 9.8);
  Eigen::Vector3f FCero(0, 0, -Mcss * 9.8);
  Eigen::Vector3f FDero(0, 0, -Mdss * 9.8);
  Eigen::Vector3f FEero(0, 0, -Mess * 9.8);

  Eigen::Vector3f Ga(0, 0, Mass * 9.8);
  Eigen::Vector3f Gb(0, 0, Mbss * 9.8);
  Eigen::Vector3f Gc(0, 0, Mcss * 9.8);
  Eigen::Vector3f Gd(0, 0, Mdss * 9.8);
  Eigen::Vector3f Ge(0, 0, Mess * 9.8);

  Eigen::Matrix3f Eia = Eigen::Matrix3f::Identity();

  // 计算 F1x
  Eigen::Vector3f F1x = Eia * (Ga + Gb + Gc + Gd + Ge) + FAero 
                      + Eab.transpose() * FBero 
                      + Eac.transpose() * FCero 
                      + Eab.transpose() * Ebd.transpose() * FDero 
                      + Eac.transpose() * Ece.transpose() * FEero;

  // 计算 M1x
  Eigen::Vector3f M1x = (Rab + Eab.transpose() * Rbm).cross(Eia * Gb) 
                      + (Rab + Eab.transpose() * (Rbm + Rbd) + Eab.transpose() * Ebd.transpose() * Rdm).cross(Eia * Gd)
                      + (Rac + Eac.transpose() * Rcm).cross(Eia * Gc) 
                      + (Rac + Eac.transpose() * (Rcm + Rce) + Eac.transpose() * Ece.transpose() * Rem).cross(Eia * Ge)
                      + (Rab + Eab.transpose() * Rbm).cross(Eab.transpose() * FBero) 
                      + (Rab + Eab.transpose() * (Rbm + Rbd) + Eab.transpose() * Ebd.transpose() * Rdm).cross(Eab.transpose() * Ebd.transpose() * FDero)
                      + (Rac + Eac.transpose() * Rcm).cross(Eac.transpose() * FCero) 
                      + (Rac + Eac.transpose() * (Rcm + Rce) + Eac.transpose() * Ece.transpose() * Rem).cross(Eac.transpose() * Ece.transpose() * FEero);

  // 计算 M2x
  Eigen::Vector3f M2x =  (Rbm + Rbd + Ebd.transpose() * Rdm).cross(Eab * Eia * Gd)
                      +  Rbm.cross(Eab * Eia * Gb)
                      +  Rbm.cross(FBero)
                      +  (Rbm + Rbd + Ebd.transpose() * Rdm).cross(Ebd.transpose() * FDero);

  // 计算 M3x
  Eigen::Vector3f M3x = (Rcm + Rce + Ece.transpose() * Rem).cross(Eac * Eia * Ge)
                      +  Rcm.cross(Eac * Eia * Gc)
                      +  Rcm.cross(FCero)
                      +  (Rcm + Rce + Ece.transpose() * Rem).cross(Ece.transpose() * FEero);

  // 计算 M4x
  Eigen::Vector3f M4x =  Rdm.cross(Ebd * Eab * Eia * Gd)
                      +  Rdm.cross(FDero);

  // 计算 M5x
  Eigen::Vector3f M5x =  Rem.cross(Ece * Eac * Eia * Ge)
                      +  Rem.cross(FEero);

  // 组合结果向量 Qx
  Eigen::Matrix<float, 10, 1> Qx;
  Qx << F1x, M1x, M2x.row(0), M3x.row(0), M4x.row(0), M5x.row(0);

  return Qx;
}



void getpinvBplusmini() {

    MatrixXf M = assembleMassMatrix();

    MatrixXf Be = computeBeMatrix(y, relativeAngle_ready*0.01745, phiac*0.01745, phibd*0.01745, phice*0.01745);
    extractMpi(); //得到Mpi
    MatrixXf Mpi_pinv = Mpi.inverse();  // 实际计算
    getQx();
    dw_rev=-Mpi_pinv*Qx;

    
    // 打印验证部分
    MatrixXf Bplus = Mpi_pinv * Be * H;
    
    //只考虑特定的状态量
    Bplusmini << Bplus.row(6),   // MATLAB第7行 → C++第6行
               Bplus.row(7),   // MATLAB第8行 → C++第7行
               Bplus.row(8),  // MATLAB第9行 → C++第8行
               Bplus.row(9);  // MATLAB第10行 → C++第9行
    
    
    Bplusfull << Bplus.row(3),  // MATLAB第16行 → C++第15行
               Bplus.row(5),   // MATLAB第10行 → C++第9行
               Bplus.row(6),  // MATLAB第13行 → C++第12行
               Bplus.row(7),  // MATLAB第16行 → C++第15行
               Bplus.row(8),  // MATLAB第13行 → C++第12行
               Bplus.row(9);  // MATLAB第16行 → C++第15行
    
    //只考虑所有的副翼
    Bplusminismall << Bplusmini.col(0),Bplusmini.col(1),Bplusmini.col(4),Bplusmini.col(5),Bplusmini.col(8),Bplusmini.col(9),Bplusmini.col(12),Bplusmini.col(13),Bplusmini.col(16),Bplusmini.col(17);
    Bplusfullsmall << Bplusfull.col(0),Bplusfull.col(1),Bplusfull.col(4),Bplusfull.col(5),Bplusfull.col(8),Bplusfull.col(9),Bplusfull.col(12),Bplusfull.col(13),Bplusfull.col(16),Bplusfull.col(17);
    
    Bplusfullsmall_pinv=pseudoInverse(Bplusfullsmall);  // 实际计算
    Bplusminismall_pinv=pseudoInverse(Bplusminismall);  // 实际计算
    
    //printFullMatrix(Bplusmini_pinv);
    //printFullMatrix(Mpi);
}




void printFullMatrix(const MatrixXf& m) {
    
    Serial.println("==========================================");
    
    for (int i = 0; i < m.rows(); i++) {
        // 行号标签
        Serial.print("[");
        if (i < 10) Serial.print(" ");
        Serial.print(i);
        Serial.print("] ");
        
        // 打印每行元素
        for (int j = 0; j < m.cols(); j++) {
            Serial.print(m(i,j), 4); // 打印4位小数
            Serial.print("\t");
            
            // 每5列增加分隔线
            if ((j+1) % 5 == 0) Serial.print("| ");
        }
        Serial.println();
        
        // 每5行增加分隔线
        if ((i+1) % 5 == 0) {
            Serial.println("------------------------------------------");
        }
    }
    Serial.println("==========================================");
}


// 优化的伪逆计算函数
Eigen::MatrixXf pseudoInverse(const Eigen::MatrixXf &a) {
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  const Eigen::VectorXf& singularValues = svd.singularValues();
  Eigen::VectorXf singularValuesInv(singularValues.size());
  
  float tolerance = 1e-6f * std::max(a.rows(), a.cols()) * singularValues.array().abs()(0);
  
  for (int i = 0; i < singularValues.size(); ++i) {
    singularValuesInv(i) = (singularValues(i) > tolerance) ? 1.0f / singularValues(i) : 0.0f;
  }
  
  return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().adjoint();
}


void initializeH()
{
 // 按行初始化，所有数值保留1位小数
  H << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       36.0, 40.6, 21.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 12.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 42.0, 42.8, 21.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 43.0, 43.0, 21.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 42.8, 42.0, 21.6, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.3, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.6, 36.0, 21.6, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.3,
       8.0, -10.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, -15.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 10.7, -10.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.9, -10.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.8, -10.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.6, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.4, -8.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.6, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}


void extractMpi()
 {
  Mp << M.row(0),   // 第1行 (MATLAB的M(1,:))
         M.row(1),   // 第2行
         M.row(2),   // 第3行
         M.row(3),   // 第4行
         M.row(4),   // 第5行
         M.row(5),   // 第6行
         M.row(6),   // 第7行
         M.row(9),   // 第10行
         M.row(12),  // 第13行
         M.row(15);  // 第16行
Mpi<< Mp.col(0),Mp.col(1),Mp.col(2),Mp.col(3),Mp.col(4),Mp.col(5),Mp.col(6),Mp.col(9),Mp.col(12),Mp.col(15);
  

}


MatrixXf computeBeMatrix(float y, float phiab, float phiac, float phibd, float phice) {
    // 初始化所有子矩阵
    
  Matrix<float, 3, 4> BebT;
  BebT << 0,y*sin(phiab)*(1*cos(phiab)) - (y+y*cos(phiab))*(1*sin(phiab)),y*pow(sin(phiab),2)*1 + (y+y*cos(phiab))*1*cos(phiab),0,
          y*sin(phiab)*1*1,0,0,-y*sin(phiab),
          -(y+y*cos(phiab))*1*1,0,0,(y+y*cos(phiab));

  Matrix<float, 3, 4> BecT;
  BecT << 0,(y+y*cos(phiac))*(1*sin(phiac)) - y*sin(phiac)*(1*cos(phiac)),-(y+y*cos(phiac))*1*cos(phiac) - y*pow(sin(phiac),2)*1,0,
          -y*sin(phiac)*1*1,0,0,y*sin(phiac),
          (y+y*cos(phiac))*1*1,0,0,-(y+y*cos(phiac));

  // ===== 3. Compute BedT =====
  float c1 = cos(phiab)*sin(phibd) + cos(phibd)*sin(phiab);
  float c2 = cos(phiab)*cos(phibd) - sin(phiab)*sin(phibd);
  
  Matrix<float, 3, 4> BedT;
  BedT << 0,(y*c1 + 2*y*sin(phiab))*(1*c2) - (1*c1)*(y + y*c2 + 2*y*cos(phiab)),(y*c1 + 2*y*sin(phiab))*(1*c1) + (1*c2)*(y + y*c2 + 2*y*cos(phiab)),0,
          (y*c1 + 2*y*sin(phiab)),0,0,-(y*c1 + 2*y*sin(phiab)),
          -(y + y*c2 + 2*y*cos(phiab)),0,0,(y + y*c2 + 2*y*cos(phiab));

  // ===== 4. Compute BeeT =====
  float c3 = cos(phiac)*sin(phice) + cos(phice)*sin(phiac);
  float c4 = cos(phiac)*cos(phice) - sin(phiac)*sin(phice);
  
  Matrix<float, 3, 4> BeeT;
  BeeT << 0,(1*c3)*(y + y*c4 + 2*y*cos(phiac)) - (y*c3 + 2*y*sin(phiac))*(1*c4),-1*c4*(y + y*c4 + 2*y*cos(phiac)) - (y*c3 + 2*y*sin(phiac))*(1*c3),0,
          -(y*c3 + 2*y*sin(phiac))*1*1,0,0,(y*c3 + 2*y*sin(phiac)),
          (y + y*c4 + 2*y*cos(phiac)),0,0,-(y + y*c4 + 2*y*cos(phiac));

    Matrix<float, 3, 4> zerosOO = Matrix<float, 3, 4>::Zero();


    Eab << 1, 0, 0,
           0, cos(phiab), sin(phiab),
           0, -sin(phiab), cos(phiab);
    Eac << 1, 0, 0,
           0, cos(phiac), sin(phiac),
           0, -sin(phiac), cos(phiac);
    Ebd << 1, 0, 0,
           0, cos(phibd), sin(phibd),
           0, -sin(phibd), cos(phibd);
    Ece << 1, 0, 0,
           0, cos(phice), sin(phice),
           0, -sin(phice), cos(phice);
    
    // 计算行向量 (严格对应MATLAB公式)
    RowVectorXf BeLab(35), BeLac(35), BeLbd(35), BeLce(35);
    BeLab << 0, -(2*y+y*cos(phibd))*(1*sin(phibd))+y*sin(phibd)*(1*cos(phibd)), (2*y+y*cos(phibd))*1*cos(phibd)+y*pow(sin(phibd),2),0, 0, 0, y, 0,   0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0,0,    1,0,0, 1,0,0, 0,0,0, 0,0,0, 0,0,0;
    
    BeLac << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,-y,0,0,(2*y+y*cos(phice))*(1*sin(phice))-y*sin(phice)*(1*cos(phice)),-(2*y+y*cos(phice))*1*cos(phice)-y*pow(sin(phice),2)*1,0, 0,0,0, 0,0,0, 0,0,0, 1,0,0, 1,0,0;
    
    BeLbd << 0,0,y,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    
    BeLce << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-y,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0;

  
    // 直接力矩阵 (严格对应MATLAB)
    RowVectorXf BeFD(35), BeFY(35), BeFL(35);
    BeFD << 1,0,0,1, 1,0,0,1, 1,0,0,1, 1,0,0,1, 1,0,0,1,
            RowVectorXf::Zero(15);
    
    BeFY << 0, 
            (cos(phiab)*cos(phibd)-sin(phiab)*sin(phibd)),
            (cos(phiab)*sin(phibd)+cos(phibd)*sin(phiab)),
            0, 0, cos(phiab), sin(phiab), 0, 0, 1, 0, 0, 0,
            cos(phiac), sin(phiac), 0, 0,
            (cos(phiac)*cos(phice)-sin(phiac)*sin(phice)),
            (cos(phiac)*sin(phice)+cos(phice)*sin(phiac)),
            0, RowVectorXf::Zero(15);
    
    BeFL << 0,
            (cos(phiab)*sin(phibd)+cos(phibd)*sin(phiab)),
            -(cos(phiab)*cos(phibd)-sin(phiab)*sin(phibd)),
            0, 0, sin(phiab), -cos(phiab), 0, 0, 0, -1, 0, 0,
            sin(phiac), -cos(phiac), 0, 0,
            (cos(phiac)*sin(phice)+cos(phice)*sin(phiac)),
            -(cos(phiac)*cos(phice)-sin(phiac)*sin(phice)),
            0, RowVectorXf::Zero(15);

    // 组装大Be矩阵 (10x35)
    
    MatrixXf Be(10, 35);
    
    Be << BeFD,
          BeFY,
          BeFL,
          BedT, BebT, zerosOO, BecT, BeeT, (Eab.transpose()*Ebd.transpose()).eval(), Eab.transpose(), Matrix3f::Identity(), Eac.transpose(), (Eac.transpose()*Ece.transpose()).eval(),
          BeLab,
          BeLac,
          BeLbd,
          BeLce;
    
    // 置零特定列 (严格对应MATLAB)
    int zero_cols[] = {1,5,9,13,17,22,25,28,31,34};
    for (int col : zero_cols) {
        Be.col(col).setZero();
    }
    

    return Be;
}

void initializeControlAllocation() {
    initializeInertiaMatrices(Ia, Ib, Ic, Id, Ie);
    initializeH();
}
