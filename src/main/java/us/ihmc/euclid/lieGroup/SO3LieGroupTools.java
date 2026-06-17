package us.ihmc.euclid.lieGroup;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * Static utility class providing SO(3) Lie group and Lie algebra operations.
 *
 * <p>The Lie algebra so(3) is identified with ℝ³ via the hat/vee isomorphism.
 * All methods are static; {@code ToPack} parameters are the outputs — inputs are never modified.</p>
 *
 * <p>Conventions:
 * <ul>
 *   <li>hat(ω) = skew-symmetric Ω such that Ω v = ω × v</li>
 *   <li>exp(ω) maps a rotation vector (axis scaled by angle) to SO(3) via Rodrigues</li>
 *   <li>log(R) maps an SO(3) element to its rotation vector ω with ‖ω‖ ∈ [0, π]</li>
 *   <li>J_l denotes the <em>left</em> Jacobian of SO(3); J_r = J_l(-ω)</li>
 * </ul>
 * </p>
 */
public class SO3LieGroupTools
{
   /** Small-angle threshold below which series approximations are used. */
   public static final double EPS = 1.0e-7;

   private SO3LieGroupTools()
   {
   }

   // -----------------------------------------------------------------------
   // hat / vee
   // -----------------------------------------------------------------------

   /**
    * Computes the hat (skew-symmetric) map: ω ↦ Ω where Ω v = ω × v.
    *
    * <pre>
    *     [  0   -ωz   ωy ]
    * Ω = [ ωz    0   -ωx ]
    *     [-ωy   ωx    0  ]
    * </pre>
    *
    * @param omega        the rotation vector ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack the result into. Modified.
    */
   public static void hat(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      double wx = omega.getX();
      double wy = omega.getY();
      double wz = omega.getZ();
      matrixToPack.set(0.0,  -wz,   wy,
                        wz,  0.0,  -wx,
                       -wy,   wx,  0.0);
   }

   /**
    * Computes the vee map (inverse of hat): extracts ω from a skew-symmetric matrix Ω.
    *
    * @param skewMatrix   a skew-symmetric 3×3 matrix. Not modified.
    * @param vectorToPack the vector to pack ω into. Modified.
    */
   public static void vee(Matrix3DReadOnly skewMatrix, Vector3DBasics vectorToPack)
   {
      // hat(ω)[2,1] = ωx,  hat(ω)[0,2] = ωy,  hat(ω)[1,0] = ωz
      vectorToPack.set(skewMatrix.getM21(), skewMatrix.getM02(), skewMatrix.getM10());
   }

   // -----------------------------------------------------------------------
   // exp / log
   // -----------------------------------------------------------------------

   /**
    * SO(3) exponential map: converts a rotation vector ω (axis × angle) to an orientation.
    *
    * @param omega             rotation vector (axis scaled by angle). Not modified.
    * @param orientationToPack the orientation to pack the result into. Modified.
    */
   public static void exp(Vector3DReadOnly omega, Orientation3DBasics orientationToPack)
   {
      Quaternion q = new Quaternion();
      QuaternionConversion.convertRotationVectorToQuaternion(omega, q);
      orientationToPack.set(q);
   }

   /**
    * SO(3) logarithmic map: converts an orientation R to its rotation vector ω = axis × angle.
    *
    * @param rotation     the SO(3) element. Not modified.
    * @param vectorToPack the vector to pack ω into. Modified.
    */
   public static void log(Orientation3DReadOnly rotation, Vector3DBasics vectorToPack)
   {
      rotation.getRotationVector(vectorToPack);
   }

   // -----------------------------------------------------------------------
   // Adjoint representations
   // -----------------------------------------------------------------------

   /**
    * Group adjoint Ad_R: for SO(3), Ad_R = R (the rotation matrix),
    * since Ad_R ω = R ω when so(3) ≅ ℝ³.
    *
    * @param rotation     the SO(3) element. Not modified.
    * @param matrixToPack the 3×3 matrix to pack Ad_R into. Modified.
    */
   public static void adjoint(Orientation3DReadOnly rotation, Matrix3DBasics matrixToPack)
   {
      matrixToPack.set(rotation);
   }

   /**
    * Algebra adjoint (small adjoint) ad_ω: the Lie bracket map ξ ↦ ω × ξ on so(3).
    * ad_ω = hat(ω).
    *
    * @param omega        the algebra element ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack ad_ω into. Modified.
    */
   public static void smallAdjoint(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      hat(omega, matrixToPack);
   }

   // -----------------------------------------------------------------------
   // Left / right Jacobians
   // -----------------------------------------------------------------------

   /**
    * Left Jacobian of SO(3), J_l(ω).
    *
    * <p>Closed-form (θ = ‖ω‖, n̂ = ω/θ):
    * <pre>
    *   J_l = (sinθ/θ) I  +  (1 − sinθ/θ) n̂n̂ᵀ  +  ((1−cosθ)/θ) hat(n̂)
    * </pre>
    * For θ &lt; EPS uses: J_l ≈ (1 − θ²/6) I + ½ hat(ω) + (1/6) ω ωᵀ.</p>
    *
    * @param omega        the rotation vector ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack J_l into. Modified.
    */
   public static void leftJacobian(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      double wx = omega.getX();
      double wy = omega.getY();
      double wz = omega.getZ();
      double theta2 = wx * wx + wy * wy + wz * wz;
      double theta = Math.sqrt(theta2);

      if (theta < EPS)
      {
         // J_l ≈ (1 - θ²/6) I + (1/2) hat(ω) + (1/6) ω ωᵀ
         double c6 = theta2 / 6.0;
         double s6 = 1.0 / 6.0;
         double hx = 0.5 * wx, hy = 0.5 * wy, hz = 0.5 * wz;
         matrixToPack.set(1.0 - c6 + s6 * wx * wx,        s6 * wx * wy - hz,       s6 * wx * wz + hy,
                               s6 * wy * wx + hz,    1.0 - c6 + s6 * wy * wy,       s6 * wy * wz - hx,
                               s6 * wz * wx - hy,         s6 * wz * wy + hx,  1.0 - c6 + s6 * wz * wz);
         return;
      }

      double invTheta = 1.0 / theta;
      double nx = wx * invTheta, ny = wy * invTheta, nz = wz * invTheta;

      double sinc  = Math.sin(theta) * invTheta;   // sinθ/θ
      double vers  = (1.0 - Math.cos(theta)) * invTheta; // (1 − cosθ)/θ
      double omsc  = 1.0 - sinc;                         // 1 − sinθ/θ

      // J_l = sinc·I + omsc·n̂n̂ᵀ + vers·hat(n̂)
      // hat(n̂) = [[0,-nz,ny],[nz,0,-nx],[-ny,nx,0]]
      matrixToPack.set(sinc + omsc * nx * nx,       omsc * nx * ny - vers * nz,  omsc * nx * nz + vers * ny,
                       omsc * ny * nx + vers * nz,  sinc + omsc * ny * ny,        omsc * ny * nz - vers * nx,
                       omsc * nz * nx - vers * ny,  omsc * nz * ny + vers * nx,  sinc + omsc * nz * nz);
   }

   /**
    * Right Jacobian of SO(3): J_r(ω) = J_l(−ω).
    *
    * @param omega        the rotation vector ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack J_r into. Modified.
    */
   public static void rightJacobian(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      double wx = omega.getX();
      double wy = omega.getY();
      double wz = omega.getZ();
      double theta2 = wx * wx + wy * wy + wz * wz;
      double theta = Math.sqrt(theta2);

      if (theta < EPS)
      {
         // J_r = J_l(-ω): flip hat sign
         double c6 = theta2 / 6.0;
         double s6 = 1.0 / 6.0;
         double hx = -0.5 * wx, hy = -0.5 * wy, hz = -0.5 * wz;
         matrixToPack.set(1.0 - c6 + s6 * wx * wx,        s6 * wx * wy - hz,       s6 * wx * wz + hy,
                               s6 * wy * wx + hz,    1.0 - c6 + s6 * wy * wy,       s6 * wy * wz - hx,
                               s6 * wz * wx - hy,         s6 * wz * wy + hx,  1.0 - c6 + s6 * wz * wz);
         return;
      }

      double invTheta = 1.0 / theta;
      double nx = wx * invTheta, ny = wy * invTheta, nz = wz * invTheta;

      double sinc = Math.sin(theta) * invTheta;
      double vers = (1.0 - Math.cos(theta)) * invTheta;
      double omsc = 1.0 - sinc;

      // J_r = J_l(-ω): flip the hat(n̂) term sign
      matrixToPack.set(sinc + omsc * nx * nx,       omsc * nx * ny + vers * nz,  omsc * nx * nz - vers * ny,
                       omsc * ny * nx - vers * nz,  sinc + omsc * ny * ny,        omsc * ny * nz + vers * nx,
                       omsc * nz * nx + vers * ny,  omsc * nz * ny - vers * nx,  sinc + omsc * nz * nz);
   }

   /**
    * Inverse of the left Jacobian of SO(3): J_l⁻¹(ω).
    *
    * <p>Closed-form (θ = ‖ω‖, n̂ = ω/θ):
    * <pre>
    *   J_l⁻¹ = (θ/2) cot(θ/2) I  +  (1 − (θ/2) cot(θ/2)) n̂n̂ᵀ  −  (θ/2) hat(n̂)
    * </pre>
    * For θ &lt; EPS uses: J_l⁻¹ ≈ (1 − θ²/12) I − ½ hat(ω) + (1/12) ω ωᵀ.</p>
    *
    * @param omega        the rotation vector ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack J_l⁻¹ into. Modified.
    */
   public static void leftJacobianInverse(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      double wx = omega.getX();
      double wy = omega.getY();
      double wz = omega.getZ();
      double theta2 = wx * wx + wy * wy + wz * wz;
      double theta = Math.sqrt(theta2);

      if (theta < EPS)
      {
         // J_l⁻¹ ≈ (1 − θ²/12) I − (1/2) hat(ω) + (1/12) ω ωᵀ
         double c12 = theta2 / 12.0;
         double t12 = 1.0 / 12.0;
         double hx = -0.5 * wx, hy = -0.5 * wy, hz = -0.5 * wz;
         matrixToPack.set(1.0 - c12 + t12 * wx * wx,        t12 * wx * wy - hz,       t12 * wx * wz + hy,
                               t12 * wy * wx + hz,    1.0 - c12 + t12 * wy * wy,       t12 * wy * wz - hx,
                               t12 * wz * wx - hy,         t12 * wz * wy + hx,  1.0 - c12 + t12 * wz * wz);
         return;
      }

      double invTheta = 1.0 / theta;
      double nx = wx * invTheta, ny = wy * invTheta, nz = wz * invTheta;

      double halfTheta   = 0.5 * theta;
      double htCot       = halfTheta * (Math.cos(halfTheta) / Math.sin(halfTheta)); // (θ/2)cot(θ/2)
      double omhtc       = 1.0 - htCot;                                             // 1 − (θ/2)cot(θ/2)

      // J_l⁻¹ = htCot·I + omhtc·n̂n̂ᵀ − halfTheta·hat(n̂)
      // −halfTheta·hat(n̂): negate the hat entries, then multiply by halfTheta
      // hat(n̂)[0,1] = -nz  → -(−halfTheta)(−nz) = −halfTheta·(−nz) = +halfTheta·nz
      // i.e. contribution at (0,1) = +halfTheta*nz, at (1,0) = -halfTheta*nz
      matrixToPack.set(htCot + omhtc * nx * nx,       omhtc * nx * ny + halfTheta * nz,  omhtc * nx * nz - halfTheta * ny,
                       omhtc * ny * nx - halfTheta * nz,  htCot + omhtc * ny * ny,        omhtc * ny * nz + halfTheta * nx,
                       omhtc * nz * nx + halfTheta * ny,  omhtc * nz * ny - halfTheta * nx,  htCot + omhtc * nz * nz);
   }

   /**
    * Inverse of the right Jacobian of SO(3): J_r⁻¹(ω) = J_l⁻¹(−ω).
    *
    * @param omega        the rotation vector ω. Not modified.
    * @param matrixToPack the 3×3 matrix to pack J_r⁻¹ into. Modified.
    */
   public static void rightJacobianInverse(Vector3DReadOnly omega, Matrix3DBasics matrixToPack)
   {
      double wx = omega.getX();
      double wy = omega.getY();
      double wz = omega.getZ();
      double theta2 = wx * wx + wy * wy + wz * wz;
      double theta = Math.sqrt(theta2);

      if (theta < EPS)
      {
         // J_r⁻¹ = J_l⁻¹(-ω): flip hat sign → (1 − θ²/12) I + (1/2) hat(ω) + (1/12) ω ωᵀ
         double c12 = theta2 / 12.0;
         double t12 = 1.0 / 12.0;
         double hx = 0.5 * wx, hy = 0.5 * wy, hz = 0.5 * wz;
         matrixToPack.set(1.0 - c12 + t12 * wx * wx,        t12 * wx * wy - hz,       t12 * wx * wz + hy,
                               t12 * wy * wx + hz,    1.0 - c12 + t12 * wy * wy,       t12 * wy * wz - hx,
                               t12 * wz * wx - hy,         t12 * wz * wy + hx,  1.0 - c12 + t12 * wz * wz);
         return;
      }

      double invTheta = 1.0 / theta;
      double nx = wx * invTheta, ny = wy * invTheta, nz = wz * invTheta;

      double halfTheta = 0.5 * theta;
      double htCot     = halfTheta * (Math.cos(halfTheta) / Math.sin(halfTheta));
      double omhtc     = 1.0 - htCot;

      // J_r⁻¹ = J_l⁻¹(-ω): flip the hat(n̂) sign → +halfTheta·hat(n̂)
      matrixToPack.set(htCot + omhtc * nx * nx,       omhtc * nx * ny - halfTheta * nz,  omhtc * nx * nz + halfTheta * ny,
                       omhtc * ny * nx + halfTheta * nz,  htCot + omhtc * ny * ny,        omhtc * ny * nz - halfTheta * nx,
                       omhtc * nz * nx - halfTheta * ny,  omhtc * nz * ny + halfTheta * nx,  htCot + omhtc * nz * nz);
   }
}
