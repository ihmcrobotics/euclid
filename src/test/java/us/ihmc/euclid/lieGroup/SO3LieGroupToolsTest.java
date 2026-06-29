package us.ihmc.euclid.lieGroup;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SO3LieGroupToolsTest
{
   private static final double EPSILON = 1.0e-10;
   private static final int ITERATIONS = 1000;

   // -----------------------------------------------------------------------
   // hat / vee
   // -----------------------------------------------------------------------

   @Test
   public void testHatSkewSymmetry()
   {
      Random random = new Random(1234L);
      Matrix3D mat = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random);
         SO3LieGroupTools.hat(omega, mat);

         assertEquals(0.0, mat.getM00(), EPSILON);
         assertEquals(0.0, mat.getM11(), EPSILON);
         assertEquals(0.0, mat.getM22(), EPSILON);
         assertEquals(mat.getM01(), -mat.getM10(), EPSILON);
         assertEquals(mat.getM02(), -mat.getM20(), EPSILON);
         assertEquals(mat.getM12(), -mat.getM21(), EPSILON);
      }
   }

   @Test
   public void testHatCrossProduct()
   {
      Random random = new Random(5678L);
      Matrix3D mat = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D v = EuclidCoreRandomTools.nextVector3D(random);
         SO3LieGroupTools.hat(omega, mat);

         Vector3D expected = new Vector3D();
         expected.cross(omega, v);

         double rx = mat.getM00() * v.getX() + mat.getM01() * v.getY() + mat.getM02() * v.getZ();
         double ry = mat.getM10() * v.getX() + mat.getM11() * v.getY() + mat.getM12() * v.getZ();
         double rz = mat.getM20() * v.getX() + mat.getM21() * v.getY() + mat.getM22() * v.getZ();

         assertEquals(expected.getX(), rx, EPSILON);
         assertEquals(expected.getY(), ry, EPSILON);
         assertEquals(expected.getZ(), rz, EPSILON);
      }
   }

   @Test
   public void testHatVeeRoundTrip()
   {
      Random random = new Random(9012L);
      Matrix3D mat = new Matrix3D();
      Vector3D recovered = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random);
         SO3LieGroupTools.hat(omega, mat);
         SO3LieGroupTools.vee(mat, recovered);

         assertEquals(omega.getX(), recovered.getX(), EPSILON);
         assertEquals(omega.getY(), recovered.getY(), EPSILON);
         assertEquals(omega.getZ(), recovered.getZ(), EPSILON);
      }
   }

   // -----------------------------------------------------------------------
   // exp / log round-trips
   // -----------------------------------------------------------------------

   @Test
   public void testExpLogRoundTrip_vectorToVector()
   {
      Random random = new Random(111L);
      Quaternion q = new Quaternion();
      Vector3D recovered = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextRotationVector(random);
         SO3LieGroupTools.exp(omega, q);
         SO3LieGroupTools.log(q, recovered);

         assertEquals(omega.getX(), recovered.getX(), EPSILON);
         assertEquals(omega.getY(), recovered.getY(), EPSILON);
         assertEquals(omega.getZ(), recovered.getZ(), EPSILON);
      }
   }

   @Test
   public void testExpLogRoundTrip_rotationToRotation()
   {
      Random random = new Random(222L);
      Vector3D omega = new Vector3D();
      RotationMatrix Rback = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix R = EuclidCoreRandomTools.nextRotationMatrix(random);
         SO3LieGroupTools.log(R, omega);
         SO3LieGroupTools.exp(omega, Rback);

         EuclidCoreTestTools.assertMatrix3DEquals(R, Rback, EPSILON);
      }
   }

   @Test
   public void testExpIdentity()
   {
      Vector3D zero = new Vector3D(0.0, 0.0, 0.0);
      Quaternion q = new Quaternion();
      SO3LieGroupTools.exp(zero, q);

      assertEquals(0.0, q.getX(), EPSILON);
      assertEquals(0.0, q.getY(), EPSILON);
      assertEquals(0.0, q.getZ(), EPSILON);
      assertEquals(1.0, q.getS(), EPSILON);
   }

   // -----------------------------------------------------------------------
   // adjoint / smallAdjoint
   // -----------------------------------------------------------------------

   @Test
   public void testAdjointEqualsRotationMatrix()
   {
      Random random = new Random(333L);
      Matrix3D ad = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix R = EuclidCoreRandomTools.nextRotationMatrix(random);
         SO3LieGroupTools.adjoint(R, ad);
         EuclidCoreTestTools.assertMatrix3DEquals(R, ad, EPSILON);
      }
   }

   @Test
   public void testSmallAdjointEqualsHat()
   {
      Random random = new Random(444L);
      Matrix3D adSmall = new Matrix3D();
      Matrix3D hatM = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random);
         SO3LieGroupTools.smallAdjoint(omega, adSmall);
         SO3LieGroupTools.hat(omega, hatM);
         EuclidCoreTestTools.assertMatrix3DEquals(hatM, adSmall, EPSILON);
      }
   }

   // -----------------------------------------------------------------------
   // Jacobians
   // -----------------------------------------------------------------------

   @Test
   public void testJacobianNearZeroFirstOrder()
   {
      // For small ω: J_l ≈ I + ½ hat(ω)  (first-order)
      Random random = new Random(555L);
      Matrix3D Jl = new Matrix3D();
      Matrix3D hatM = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextVector3D(random, 1e-5);
         SO3LieGroupTools.leftJacobian(omega, Jl);
         SO3LieGroupTools.hat(omega, hatM);

         double tol = 1e-9; // second-order terms are O(θ²) ≈ O(1e-10)
         assertEquals(1.0, Jl.getM00(), tol);
         assertEquals(1.0, Jl.getM11(), tol);
         assertEquals(1.0, Jl.getM22(), tol);
         assertEquals(0.5 * hatM.getM01(), Jl.getM01(), tol);
         assertEquals(0.5 * hatM.getM10(), Jl.getM10(), tol);
      }
   }

   @Test
   public void testJacobiansLeftRightRelation()
   {
      // J_r(ω) = J_l(-ω)
      Random random = new Random(666L);
      Matrix3D Jl = new Matrix3D();
      Matrix3D Jr = new Matrix3D();
      Matrix3D JlNeg = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextRotationVector(random);
         SO3LieGroupTools.leftJacobian(omega, Jl);
         SO3LieGroupTools.rightJacobian(omega, Jr);

         Vector3D omegaNeg = new Vector3D(-omega.getX(), -omega.getY(), -omega.getZ());
         SO3LieGroupTools.leftJacobian(omegaNeg, JlNeg);

         EuclidCoreTestTools.assertMatrix3DEquals(JlNeg, Jr, EPSILON);
      }
   }

   @Test
   public void testJacobianInverseRelations()
   {
      // J_l⁻¹ * J_l = I   and   J_r⁻¹ * J_r = I
      Random random = new Random(777L);
      Matrix3D Jl = new Matrix3D();
      Matrix3D JlInv = new Matrix3D();
      Matrix3D Jr = new Matrix3D();
      Matrix3D JrInv = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextRotationVector(random);

         SO3LieGroupTools.leftJacobian(omega, Jl);
         SO3LieGroupTools.leftJacobianInverse(omega, JlInv);
         assertMatrix3DProductIsIdentity(JlInv, Jl, 1.0e-9);

         SO3LieGroupTools.rightJacobian(omega, Jr);
         SO3LieGroupTools.rightJacobianInverse(omega, JrInv);
         assertMatrix3DProductIsIdentity(JrInv, Jr, 1.0e-9);
      }
   }

   @Test
   public void testRightJacobianInverseRelation()
   {
      // J_r⁻¹(ω) = J_l⁻¹(-ω)
      Random random = new Random(888L);
      Matrix3D JrInv = new Matrix3D();
      Matrix3D JlInvNeg = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextRotationVector(random);
         SO3LieGroupTools.rightJacobianInverse(omega, JrInv);

         Vector3D omegaNeg = new Vector3D(-omega.getX(), -omega.getY(), -omega.getZ());
         SO3LieGroupTools.leftJacobianInverse(omegaNeg, JlInvNeg);

         EuclidCoreTestTools.assertMatrix3DEquals(JlInvNeg, JrInv, EPSILON);
      }
   }

   @Test
   public void testJacobianJlTransposeEqualsJr()
   {
      // For SO(3), J_r(ω) = J_l(ω)ᵀ
      Random random = new Random(999L);
      Matrix3D Jl = new Matrix3D();
      Matrix3D Jr = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D omega = EuclidCoreRandomTools.nextRotationVector(random);
         SO3LieGroupTools.leftJacobian(omega, Jl);
         SO3LieGroupTools.rightJacobian(omega, Jr);

         assertEquals(Jl.getM00(), Jr.getM00(), EPSILON);
         assertEquals(Jl.getM01(), Jr.getM10(), EPSILON);
         assertEquals(Jl.getM02(), Jr.getM20(), EPSILON);
         assertEquals(Jl.getM10(), Jr.getM01(), EPSILON);
         assertEquals(Jl.getM11(), Jr.getM11(), EPSILON);
         assertEquals(Jl.getM12(), Jr.getM21(), EPSILON);
         assertEquals(Jl.getM20(), Jr.getM02(), EPSILON);
         assertEquals(Jl.getM21(), Jr.getM12(), EPSILON);
         assertEquals(Jl.getM22(), Jr.getM22(), EPSILON);
      }
   }

   @Test
   public void testGamma2AtZero()
   {
      // Γ₂(0) = Σ φ̂ⁿ/(n+2)! evaluated at 0 = I/2! = ½I
      Matrix3D gamma2 = new Matrix3D();
      SO3LieGroupTools.gamma2(new Vector3D(0.0, 0.0, 0.0), gamma2);

     Matrix3D halfIdentity = new Matrix3D();
     halfIdentity.setIdentity();
     halfIdentity.scale(0.5);

     EuclidCoreTestTools.assertMatrix3DEquals(halfIdentity, gamma2, EPSILON);
   }

   @Test
   public void testGamma2RecurrenceWithLeftJacobian()
   {
      // Recurrence Γ₁(φ) = I + hat(φ)·Γ₂(φ), with Γ₁ = leftJacobian.
      Random random = new Random(2024L);
      Matrix3D gamma2 = new Matrix3D();
      Matrix3D hatPhi = new Matrix3D();
      Matrix3D Jl = new Matrix3D();
      Matrix3D reconstructed = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random);
         SO3LieGroupTools.gamma2(phi,gamma2);
         SO3LieGroupTools.hat(phi, hatPhi);
         SO3LieGroupTools.leftJacobian(phi, Jl);


         // reconstructed = I + hatPhi * gamma2
         for (int r = 0; r < 3; r++)
         {
            for (int c = 0; c < 3; c++)
            {
               double sum = (r == c) ? 1.0 : 0.0;
               for (int k = 0; k < 3; k++)
                  sum += hatPhi.getElement(r,k) * gamma2.getElement(k,c);
               reconstructed.setElement(r,c,sum);
            }
         }
         EuclidCoreTestTools.assertMatrix3DEquals(Jl, reconstructed, 1.0e-9);
      }
   }

   @Test
   public void testGamma2SmallAngleRecurrence()
   {
      // Same recurrence but with the small angle assumption, to try out the Taylor series and make sure that it works
      Random random = new Random(4096L);
      Matrix3D gamma2 = new Matrix3D();
      Matrix3D hatPhi = new Matrix3D();
      Matrix3D Jl = new Matrix3D();
      Matrix3D reconstructed = new Matrix3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random, 1.0e-8);
         SO3LieGroupTools.gamma2(phi,gamma2);
         SO3LieGroupTools.hat(phi, hatPhi);
         SO3LieGroupTools.leftJacobian(phi, Jl);

         for (int r = 0; r < 3; r++)
         {
            for (int c = 0; c < 3; c++)
            {
               double sum = (r == c) ? 1.0 : 0.0;
               for (int k = 0; k < 3; k++)
                  sum += hatPhi.getElement(r,k) * gamma2.getElement(k,c);
               reconstructed.setElement(r,c,sum);
            }
         }
         EuclidCoreTestTools.assertMatrix3DEquals(Jl, reconstructed, 1.0e-12);
      }
   }

   // -----------------------------------------------------------------------
   // helpers
   // -----------------------------------------------------------------------

   private static void assertMatrix3DProductIsIdentity(Matrix3D A, Matrix3D B, double epsilon)
   {
      for (int r = 0; r < 3; r++)
      {
         for (int c = 0; c < 3; c++)
         {
            double sum = 0.0;
            for (int k = 0; k < 3; k++)
               sum += A.getElement(r, k) * B.getElement(k, c);
            double expected = (r == c) ? 1.0 : 0.0;
            assertEquals(expected, sum, epsilon, "A*B element (" + r + "," + c + ")");
         }
      }
   }
}
