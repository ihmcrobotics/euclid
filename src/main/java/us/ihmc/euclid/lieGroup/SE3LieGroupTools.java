package us.ihmc.euclid.lieGroup;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Static utility class providing SE(3) Lie group and Lie algebra operations.
 *
 * <p>The Lie algebra se(3) is identified with ℝ⁶ via the hat/vee isomorphism.
 * The 6-vector convention is {@code ξ = [φ; ρ]} where:
 * <ul>
 *   <li>φ = xi[0..2] — rotational (angular) component</li>
 *   <li>ρ = xi[3..5] — translational (linear) component</li>
 * </ul>
 * </p>
 *
 * <p>The hat map produces a 4×4 matrix:
 * <pre>
 *            [ hat(φ)   ρ ]
 *   hat(ξ) = [            ]
 *            [  0  0  0 0 ]
 * </pre>
 * </p>
 *
 * <p>The SE(3) group adjoint (6×6) for T = (R, t) with 6-vector ordering [φ; ρ] is:
 * <pre>
 *   Ad_T = [    R     0 ]
 *          [ hat(t)·R R ]
 * </pre>
 * </p>
 *
 * <p>Allocation note: {@link #exp}, {@link #log}, {@link #adjoint}, and
 * {@link #smallAdjoint} allocate small temporaries. They are <em>not</em> suitable
 * for hard real-time loops without pre-allocation wrappers.</p>
 */
public class SE3LieGroupTools
{
   private SE3LieGroupTools()
   {
   }

   // -----------------------------------------------------------------------
   // hat / vee
   // -----------------------------------------------------------------------

   /**
    * SE(3) hat map: converts a 6-vector ξ = [φ; ρ] to a 4×4 se(3) matrix.
    *
    * <pre>
    * xi[0..2] = φ (rotation part),  xi[3..5] = ρ (translation part)
    *
    *            [  0   -φz   φy   ρx ]
    *            [ φz    0   -φx   ρy ]
    * hat(ξ) =   [-φy   φx    0    ρz ]
    *            [  0    0    0     0  ]
    * </pre>
    *
    * @param xi          6-element array [φx, φy, φz, ρx, ρy, ρz]. Not modified.
    * @param matrixToPack 4×4 matrix to pack the result into (must be at least 4×4). Modified.
    */
   public static void hat(double[] xi, DMatrixRMaj matrixToPack)
   {
      double phiX = xi[0], phiY = xi[1], phiZ = xi[2];
      double rhoX = xi[3], rhoY = xi[4], rhoZ = xi[5];

      matrixToPack.unsafe_set(0, 0, 0.0);
      matrixToPack.unsafe_set(0, 1, -phiZ);
      matrixToPack.unsafe_set(0, 2, phiY);
      matrixToPack.unsafe_set(0, 3, rhoX);

      matrixToPack.unsafe_set(1, 0, phiZ);
      matrixToPack.unsafe_set(1, 1, 0.0);
      matrixToPack.unsafe_set(1, 2, -phiX);
      matrixToPack.unsafe_set(1, 3, rhoY);

      matrixToPack.unsafe_set(2, 0, -phiY);
      matrixToPack.unsafe_set(2, 1, phiX);
      matrixToPack.unsafe_set(2, 2, 0.0);
      matrixToPack.unsafe_set(2, 3, rhoZ);

      matrixToPack.unsafe_set(3, 0, 0.0);
      matrixToPack.unsafe_set(3, 1, 0.0);
      matrixToPack.unsafe_set(3, 2, 0.0);
      matrixToPack.unsafe_set(3, 3, 0.0);
   }

   /**
    * SE(3) vee map: extracts the 6-vector ξ = [φ; ρ] from a 4×4 se(3) hat matrix.
    *
    * @param hatMatrix  4×4 se(3) hat matrix. Not modified.
    * @param xiToPack   6-element array to pack [φx, φy, φz, ρx, ρy, ρz] into. Modified.
    */
   public static void vee(DMatrixRMaj hatMatrix, double[] xiToPack)
   {
      xiToPack[0] = hatMatrix.unsafe_get(2, 1); // φx
      xiToPack[1] = hatMatrix.unsafe_get(0, 2); // φy
      xiToPack[2] = hatMatrix.unsafe_get(1, 0); // φz
      xiToPack[3] = hatMatrix.unsafe_get(0, 3); // ρx
      xiToPack[4] = hatMatrix.unsafe_get(1, 3); // ρy
      xiToPack[5] = hatMatrix.unsafe_get(2, 3); // ρz
   }

   // -----------------------------------------------------------------------
   // exp / log
   // -----------------------------------------------------------------------

   /**
    * SE(3) exponential map: converts a Lie algebra element ξ = [φ; ρ] to a rigid-body transform.
    *
    * <p>Formula: R = exp(hat(φ)), t = J_l(φ) · ρ.</p>
    *
    * @param xi          6-element array [φx, φy, φz, ρx, ρy, ρz]. Not modified.
    * @param transformToPack the rigid-body transform to pack the result into. Modified.
    */
   public static void exp(double[] xi, RigidBodyTransformBasics transformToPack)
   {
      Vector3D phi = new Vector3D(xi[0], xi[1], xi[2]);

      RotationMatrix R = new RotationMatrix();
      SO3LieGroupTools.exp(phi, R);

      Matrix3D Jl = new Matrix3D();
      SO3LieGroupTools.leftJacobian(phi, Jl);

      double rhoX = xi[3], rhoY = xi[4], rhoZ = xi[5];
      double tx = Jl.getM00() * rhoX + Jl.getM01() * rhoY + Jl.getM02() * rhoZ;
      double ty = Jl.getM10() * rhoX + Jl.getM11() * rhoY + Jl.getM12() * rhoZ;
      double tz = Jl.getM20() * rhoX + Jl.getM21() * rhoY + Jl.getM22() * rhoZ;

      transformToPack.getRotation().set(R);
      transformToPack.getTranslation().set(tx, ty, tz);
   }

   /**
    * SE(3) logarithmic map: converts a rigid-body transform to its Lie algebra element ξ = [φ; ρ].
    *
    * <p>Formula: φ = log(R), ρ = J_l⁻¹(φ) · t.</p>
    *
    * @param transform  the rigid-body transform. Not modified.
    * @param xiToPack   6-element array to pack [φx, φy, φz, ρx, ρy, ρz] into. Modified.
    */
   public static void log(RigidBodyTransformReadOnly transform, double[] xiToPack)
   {
      Vector3D phi = new Vector3D();
      SO3LieGroupTools.log(transform.getRotation(), phi);

      Matrix3D JlInv = new Matrix3D();
      SO3LieGroupTools.leftJacobianInverse(phi, JlInv);

      double tx = transform.getTranslation().getX();
      double ty = transform.getTranslation().getY();
      double tz = transform.getTranslation().getZ();

      xiToPack[0] = phi.getX();
      xiToPack[1] = phi.getY();
      xiToPack[2] = phi.getZ();
      xiToPack[3] = JlInv.getM00() * tx + JlInv.getM01() * ty + JlInv.getM02() * tz;
      xiToPack[4] = JlInv.getM10() * tx + JlInv.getM11() * ty + JlInv.getM12() * tz;
      xiToPack[5] = JlInv.getM20() * tx + JlInv.getM21() * ty + JlInv.getM22() * tz;
   }

   // -----------------------------------------------------------------------
   // Adjoint representations
   // -----------------------------------------------------------------------

   /**
    * Group adjoint Ad_T (6×6) for T = (R, t), with 6-vector ordering [φ; ρ].
    *
    * <pre>
    *   Ad_T = [    R     0 ]
    *          [ hat(t)·R R ]
    * </pre>
    *
    * @param transform    the SE(3) element T. Not modified.
    * @param adjToPack    6×6 DMatrixRMaj to pack Ad_T into (must be at least 6×6). Modified.
    */
   public static void adjoint(RigidBodyTransformReadOnly transform, DMatrixRMaj adjToPack)
   {
      RotationMatrix R = new RotationMatrix();
      R.set(transform.getRotation());

      double tx = transform.getTranslation().getX();
      double ty = transform.getTranslation().getY();
      double tz = transform.getTranslation().getZ();

      // hat(t) = [[0, -tz, ty], [tz, 0, -tx], [-ty, tx, 0]]
      // hat(t)*R: compute rows of hat(t) dotted with cols of R
      double r00 = R.getM00(), r01 = R.getM01(), r02 = R.getM02();
      double r10 = R.getM10(), r11 = R.getM11(), r12 = R.getM12();
      double r20 = R.getM20(), r21 = R.getM21(), r22 = R.getM22();

      // [hat(t)*R]_ij = sum_k hat(t)_ik * R_kj
      // hat(t) row 0: [0, -tz, ty]
      // hat(t) row 1: [tz,  0, -tx]
      // hat(t) row 2: [-ty, tx,  0]
      double ht00 = -tz * r10 + ty * r20;
      double ht01 = -tz * r11 + ty * r21;
      double ht02 = -tz * r12 + ty * r22;
      double ht10 =  tz * r00 - tx * r20;
      double ht11 =  tz * r01 - tx * r21;
      double ht12 =  tz * r02 - tx * r22;
      double ht20 = -ty * r00 + tx * r10;
      double ht21 = -ty * r01 + tx * r11;
      double ht22 = -ty * r02 + tx * r12;

      // Top-left 3×3: R
      adjToPack.unsafe_set(0, 0, r00); adjToPack.unsafe_set(0, 1, r01); adjToPack.unsafe_set(0, 2, r02);
      adjToPack.unsafe_set(1, 0, r10); adjToPack.unsafe_set(1, 1, r11); adjToPack.unsafe_set(1, 2, r12);
      adjToPack.unsafe_set(2, 0, r20); adjToPack.unsafe_set(2, 1, r21); adjToPack.unsafe_set(2, 2, r22);

      // Top-right 3×3: 0
      adjToPack.unsafe_set(0, 3, 0.0); adjToPack.unsafe_set(0, 4, 0.0); adjToPack.unsafe_set(0, 5, 0.0);
      adjToPack.unsafe_set(1, 3, 0.0); adjToPack.unsafe_set(1, 4, 0.0); adjToPack.unsafe_set(1, 5, 0.0);
      adjToPack.unsafe_set(2, 3, 0.0); adjToPack.unsafe_set(2, 4, 0.0); adjToPack.unsafe_set(2, 5, 0.0);

      // Bottom-left 3×3: hat(t)*R
      adjToPack.unsafe_set(3, 0, ht00); adjToPack.unsafe_set(3, 1, ht01); adjToPack.unsafe_set(3, 2, ht02);
      adjToPack.unsafe_set(4, 0, ht10); adjToPack.unsafe_set(4, 1, ht11); adjToPack.unsafe_set(4, 2, ht12);
      adjToPack.unsafe_set(5, 0, ht20); adjToPack.unsafe_set(5, 1, ht21); adjToPack.unsafe_set(5, 2, ht22);

      // Bottom-right 3×3: R
      adjToPack.unsafe_set(3, 3, r00); adjToPack.unsafe_set(3, 4, r01); adjToPack.unsafe_set(3, 5, r02);
      adjToPack.unsafe_set(4, 3, r10); adjToPack.unsafe_set(4, 4, r11); adjToPack.unsafe_set(4, 5, r12);
      adjToPack.unsafe_set(5, 3, r20); adjToPack.unsafe_set(5, 4, r21); adjToPack.unsafe_set(5, 5, r22);
   }

   /**
    * Algebra adjoint (small adjoint) ad_ξ (6×6) for ξ = [φ; ρ].
    *
    * <pre>
    *   ad_ξ = [ hat(φ)     0    ]
    *          [ hat(ρ)   hat(φ) ]
    * </pre>
    *
    * @param xi         6-element array [φx, φy, φz, ρx, ρy, ρz]. Not modified.
    * @param adToPack   6×6 DMatrixRMaj to pack ad_ξ into (must be at least 6×6). Modified.
    */
   public static void smallAdjoint(double[] xi, DMatrixRMaj adToPack)
   {
      double phiX = xi[0], phiY = xi[1], phiZ = xi[2];
      double rhoX = xi[3], rhoY = xi[4], rhoZ = xi[5];

      // hat(phi) = [[0, -phiZ, phiY], [phiZ, 0, -phiX], [-phiY, phiX, 0]]
      // hat(rho) = [[0, -rhoZ, rhoY], [rhoZ, 0, -rhoX], [-rhoY, rhoX, 0]]

      // Top-left 3×3: hat(phi)
      adToPack.unsafe_set(0, 0, 0.0);    adToPack.unsafe_set(0, 1, -phiZ); adToPack.unsafe_set(0, 2, phiY);
      adToPack.unsafe_set(1, 0, phiZ);   adToPack.unsafe_set(1, 1, 0.0);   adToPack.unsafe_set(1, 2, -phiX);
      adToPack.unsafe_set(2, 0, -phiY);  adToPack.unsafe_set(2, 1, phiX);  adToPack.unsafe_set(2, 2, 0.0);

      // Top-right 3×3: 0
      adToPack.unsafe_set(0, 3, 0.0); adToPack.unsafe_set(0, 4, 0.0); adToPack.unsafe_set(0, 5, 0.0);
      adToPack.unsafe_set(1, 3, 0.0); adToPack.unsafe_set(1, 4, 0.0); adToPack.unsafe_set(1, 5, 0.0);
      adToPack.unsafe_set(2, 3, 0.0); adToPack.unsafe_set(2, 4, 0.0); adToPack.unsafe_set(2, 5, 0.0);

      // Bottom-left 3×3: hat(rho)
      adToPack.unsafe_set(3, 0, 0.0);    adToPack.unsafe_set(3, 1, -rhoZ); adToPack.unsafe_set(3, 2, rhoY);
      adToPack.unsafe_set(4, 0, rhoZ);   adToPack.unsafe_set(4, 1, 0.0);   adToPack.unsafe_set(4, 2, -rhoX);
      adToPack.unsafe_set(5, 0, -rhoY);  adToPack.unsafe_set(5, 1, rhoX);  adToPack.unsafe_set(5, 2, 0.0);

      // Bottom-right 3×3: hat(phi)
      adToPack.unsafe_set(3, 3, 0.0);    adToPack.unsafe_set(3, 4, -phiZ); adToPack.unsafe_set(3, 5, phiY);
      adToPack.unsafe_set(4, 3, phiZ);   adToPack.unsafe_set(4, 4, 0.0);   adToPack.unsafe_set(4, 5, -phiX);
      adToPack.unsafe_set(5, 3, -phiY);  adToPack.unsafe_set(5, 4, phiX);  adToPack.unsafe_set(5, 5, 0.0);
   }
}
