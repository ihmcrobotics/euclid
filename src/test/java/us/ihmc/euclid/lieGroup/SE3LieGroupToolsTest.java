package us.ihmc.euclid.lieGroup;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SE3LieGroupToolsTest
{
   private static final double EPSILON = 1.0e-10;
   private static final int ITERATIONS = 1000;

   // -----------------------------------------------------------------------
   // hat / vee
   // -----------------------------------------------------------------------

   @Test
   public void testHatStructure()
   {
      Random random = new Random(1234L);
      DMatrixRMaj hat = new DMatrixRMaj(4, 4);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] xi = randomXi(random);
         SE3LieGroupTools.hat(xi, hat);

         double phiX = xi[0], phiY = xi[1], phiZ = xi[2];
         double rhoX = xi[3], rhoY = xi[4], rhoZ = xi[5];

         // Last row must be zero
         for (int c = 0; c < 4; c++)
            assertEquals(0.0, hat.unsafe_get(3, c), EPSILON, "hat[3," + c + "]");

         // Diagonal of hat(phi) block is zero
         assertEquals(0.0, hat.unsafe_get(0, 0), EPSILON);
         assertEquals(0.0, hat.unsafe_get(1, 1), EPSILON);
         assertEquals(0.0, hat.unsafe_get(2, 2), EPSILON);

         // Translation column
         assertEquals(rhoX, hat.unsafe_get(0, 3), EPSILON);
         assertEquals(rhoY, hat.unsafe_get(1, 3), EPSILON);
         assertEquals(rhoZ, hat.unsafe_get(2, 3), EPSILON);

         // hat(phi) off-diagonals
         assertEquals(-phiZ, hat.unsafe_get(0, 1), EPSILON);
         assertEquals( phiY, hat.unsafe_get(0, 2), EPSILON);
         assertEquals( phiZ, hat.unsafe_get(1, 0), EPSILON);
         assertEquals(-phiX, hat.unsafe_get(1, 2), EPSILON);
         assertEquals(-phiY, hat.unsafe_get(2, 0), EPSILON);
         assertEquals( phiX, hat.unsafe_get(2, 1), EPSILON);
      }
   }

   @Test
   public void testHatVeeRoundTrip()
   {
      Random random = new Random(5678L);
      DMatrixRMaj hat = new DMatrixRMaj(4, 4);
      double[] xiRecovered = new double[6];

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] xi = randomXi(random);
         SE3LieGroupTools.hat(xi, hat);
         SE3LieGroupTools.vee(hat, xiRecovered);

         for (int k = 0; k < 6; k++)
            assertEquals(xi[k], xiRecovered[k], EPSILON, "xi[" + k + "]");
      }
   }

   // -----------------------------------------------------------------------
   // exp / log round-trips
   // -----------------------------------------------------------------------

   @Test
   public void testExpLogRoundTrip()
   {
      Random random = new Random(111L);
      RigidBodyTransform T = new RigidBodyTransform();
      double[] xiRecovered = new double[6];

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] xi = randomSmallXi(random);
         SE3LieGroupTools.exp(xi, T);
         SE3LieGroupTools.log(T, xiRecovered);

         for (int k = 0; k < 6; k++)
            assertEquals(xi[k], xiRecovered[k], EPSILON, "xi[" + k + "]");
      }
   }

   @Test
   public void testLogExpRoundTrip()
   {
      Random random = new Random(222L);
      RigidBodyTransform Tback = new RigidBodyTransform();
      double[] xi = new double[6];

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform T = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         SE3LieGroupTools.log(T, xi);
         SE3LieGroupTools.exp(xi, Tback);

         EuclidCoreTestTools.assertMatrix3DEquals(T.getRotation(), Tback.getRotation(), EPSILON);
         assertEquals(T.getTranslationX(), Tback.getTranslationX(), EPSILON);
         assertEquals(T.getTranslationY(), Tback.getTranslationY(), EPSILON);
         assertEquals(T.getTranslationZ(), Tback.getTranslationZ(), EPSILON);
      }
   }

   @Test
   public void testExpPureRotation()
   {
      // exp([0; phi]) rotation must match SO(3) exp; translation must be zero
      Random random = new Random(333L);
      RigidBodyTransform T = new RigidBodyTransform();
      RotationMatrix Rexpected = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random);
         double[] xi = {phi.getX(), phi.getY(), phi.getZ(), 0.0, 0.0, 0.0};
         SE3LieGroupTools.exp(xi, T);

         SO3LieGroupTools.exp(phi, Rexpected);
         EuclidCoreTestTools.assertMatrix3DEquals(Rexpected, T.getRotation(), EPSILON);
         assertEquals(0.0, T.getTranslationX(), EPSILON);
         assertEquals(0.0, T.getTranslationY(), EPSILON);
         assertEquals(0.0, T.getTranslationZ(), EPSILON);
      }
   }

   @Test
   public void testExpPureTranslation()
   {
      // exp([rho; 0]) = (I, rho)
      Random random = new Random(444L);
      RigidBodyTransform T = new RigidBodyTransform();
      RotationMatrix identity = new RotationMatrix();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D rho = EuclidCoreRandomTools.nextVector3D(random);
         double[] xi = {0.0, 0.0, 0.0, rho.getX(), rho.getY(), rho.getZ()};
         SE3LieGroupTools.exp(xi, T);

         EuclidCoreTestTools.assertMatrix3DEquals(identity, T.getRotation(), EPSILON);
         assertEquals(rho.getX(), T.getTranslationX(), EPSILON);
         assertEquals(rho.getY(), T.getTranslationY(), EPSILON);
         assertEquals(rho.getZ(), T.getTranslationZ(), EPSILON);
      }
   }

   @Test
   public void testExpIdentity()
   {
      double[] zero = new double[6];
      RigidBodyTransform T = new RigidBodyTransform();
      SE3LieGroupTools.exp(zero, T);

      RotationMatrix identity = new RotationMatrix();
      EuclidCoreTestTools.assertMatrix3DEquals(identity, T.getRotation(), EPSILON);
      assertEquals(0.0, T.getTranslationX(), EPSILON);
      assertEquals(0.0, T.getTranslationY(), EPSILON);
      assertEquals(0.0, T.getTranslationZ(), EPSILON);
   }

   // -----------------------------------------------------------------------
   // Adjoint
   // -----------------------------------------------------------------------

   @Test
   public void testAdjointGroupHomomorphism()
   {
      // Ad_{T1 * T2} = Ad_{T1} * Ad_{T2}
      Random random = new Random(555L);
      DMatrixRMaj adT1 = new DMatrixRMaj(6, 6);
      DMatrixRMaj adT2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj adT1T2 = new DMatrixRMaj(6, 6);
      DMatrixRMaj product = new DMatrixRMaj(6, 6);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform T1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform T2 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform T1T2 = new RigidBodyTransform(T1);
         T1T2.multiply(T2);

         SE3LieGroupTools.adjoint(T1, adT1);
         SE3LieGroupTools.adjoint(T2, adT2);
         SE3LieGroupTools.adjoint(T1T2, adT1T2);

         CommonOps_DDRM.mult(adT1, adT2, product);

         assertDMatrixEquals(adT1T2, product, 1.0e-8);
      }
   }

   @Test
   public void testAdjointIdentity()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      DMatrixRMaj ad = new DMatrixRMaj(6, 6);
      SE3LieGroupTools.adjoint(identity, ad);

      for (int r = 0; r < 6; r++)
         for (int c = 0; c < 6; c++)
            assertEquals(r == c ? 1.0 : 0.0, ad.unsafe_get(r, c), EPSILON, "Ad_I[" + r + "," + c + "]");
   }

   @Test
   public void testSmallAdjointBracket()
   {
      // ad_xi1 * xi2 should equal the se(3) Lie bracket [xi1, xi2]:
      //   phi part: phi1 x phi2
      //   rho part: phi1 x rho2 + rho1 x phi2
      Random random = new Random(666L);
      DMatrixRMaj adXi1 = new DMatrixRMaj(6, 6);
      DMatrixRMaj xi2Mat = new DMatrixRMaj(6, 1);
      DMatrixRMaj bracket = new DMatrixRMaj(6, 1);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] xi1 = randomXi(random);
         double[] xi2 = randomXi(random);

         SE3LieGroupTools.smallAdjoint(xi1, adXi1);
         for (int k = 0; k < 6; k++) xi2Mat.unsafe_set(k, 0, xi2[k]);
         CommonOps_DDRM.mult(adXi1, xi2Mat, bracket);

         Vector3D phi1 = new Vector3D(xi1[0], xi1[1], xi1[2]);
         Vector3D rho1 = new Vector3D(xi1[3], xi1[4], xi1[5]);
         Vector3D phi2 = new Vector3D(xi2[0], xi2[1], xi2[2]);
         Vector3D rho2 = new Vector3D(xi2[3], xi2[4], xi2[5]);

         Vector3D expectedRho = new Vector3D();
         Vector3D tmp = new Vector3D();
         expectedRho.cross(phi1, rho2);
         tmp.cross(rho1, phi2);
         expectedRho.add(tmp);

         Vector3D expectedPhi = new Vector3D();
         expectedPhi.cross(phi1, phi2);

         assertEquals(expectedPhi.getX(), bracket.unsafe_get(0, 0), EPSILON);
         assertEquals(expectedPhi.getY(), bracket.unsafe_get(1, 0), EPSILON);
         assertEquals(expectedPhi.getZ(), bracket.unsafe_get(2, 0), EPSILON);
         assertEquals(expectedRho.getX(), bracket.unsafe_get(3, 0), EPSILON);
         assertEquals(expectedRho.getY(), bracket.unsafe_get(4, 0), EPSILON);
         assertEquals(expectedRho.getZ(), bracket.unsafe_get(5, 0), EPSILON);
      }
   }

   // -----------------------------------------------------------------------
   // helpers
   // -----------------------------------------------------------------------

   private static double[] randomXi(Random random)
   {
      double[] xi = new double[6];
      Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random);
      xi[0] = phi.getX(); xi[1] = phi.getY(); xi[2] = phi.getZ();
      for (int k = 3; k < 6; k++) xi[k] = EuclidCoreRandomTools.nextDouble(random, 5.0);
      return xi;
   }

   private static double[] randomSmallXi(Random random)
   {
      double[] xi = new double[6];
      Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random, Math.PI - 1e-3);
      xi[0] = phi.getX(); xi[1] = phi.getY(); xi[2] = phi.getZ();
      for (int k = 3; k < 6; k++) xi[k] = EuclidCoreRandomTools.nextDouble(random, 5.0);
      return xi;
   }

   private static void assertDMatrixEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      assertEquals(expected.numRows, actual.numRows);
      assertEquals(expected.numCols, actual.numCols);
      for (int r = 0; r < expected.numRows; r++)
         for (int c = 0; c < expected.numCols; c++)
            assertEquals(expected.unsafe_get(r, c), actual.unsafe_get(r, c), epsilon,
                         "Matrix[" + r + "," + c + "]");
   }
}
