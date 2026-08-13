package us.ihmc.euclid.lieGroup;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.lang.management.ManagementFactory;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SE3LieGroupToolsTest
{
   private static final double EPSILON = 1.0e-10;
   private static final int ITERATIONS = 1000;

   /** Iterations run before measuring, so JIT compilation and one-off class init are already paid. */
   private static final int WARMUP_ITERATIONS = 50_000;
   /** Measurement rounds; the minimum is taken to filter an occasional deopt/recompile blip. */
   private static final int ALLOCATION_ROUNDS = 5;
   /** Iterations per measurement round. */
   private static final int ALLOCATION_ITERATIONS = 20_000;
   /** Noise budget for a round. Expected to be 0; the allocating overloads would burn megabytes. */
   private static final long ALLOCATION_TOLERANCE_BYTES = 4096L;

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
   // allocation
   // -----------------------------------------------------------------------

   /**
    * Guards the per-tick contract: the allocation-free overloads of {@code exp}/{@code log}/
    * {@code adjoint} — the ones taking the intermediates as parameters — must not allocate.
    *
    * <p>Allocation is measured with {@code ThreadMXBean.getThreadAllocatedBytes} on this thread,
    * which counts bytes on the TLAB rather than sampling the heap, so it sees allocations even when
    * no GC runs. The loop is warmed first and the minimum across several rounds is taken.</p>
    *
    * <p>The signal is enormous relative to the tolerance: the convenience overloads allocate on the
    * order of 200 B per call, so a regression on any of the three shows up as several MB per round
    * against a 4 KB budget. The budget exists for measurement noise only — the expected value is
    * exactly 0.</p>
    */
   @Test
   public void testAllocationFree()
   {
      java.lang.management.ThreadMXBean bean = ManagementFactory.getThreadMXBean();
      assumeTrue(bean instanceof com.sun.management.ThreadMXBean, "allocation counters unavailable on this JVM");
      com.sun.management.ThreadMXBean threadBean = (com.sun.management.ThreadMXBean) bean;
      assumeTrue(threadBean.isThreadAllocatedMemorySupported(), "thread allocation measurement unsupported");
      threadBean.setThreadAllocatedMemoryEnabled(true);
      assumeTrue(threadBean.isThreadAllocatedMemoryEnabled(), "thread allocation measurement disabled");

      long threadId = Thread.currentThread().getId();

      // Everything the loop touches is allocated up front — the harness has to be silent too.
      RigidBodyTransform transform = new RigidBodyTransform();
      DMatrixRMaj adjoint = new DMatrixRMaj(6, 6);
      double[] xiIn = new double[6];
      double[] xiOut = new double[6];

      Vector3D phi = new Vector3D();
      Matrix3D jacobian = new Matrix3D();
      RotationMatrix rotation = new RotationMatrix();

      double sink = 0.0;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
         sink += driveOnce(i, xiIn, xiOut, transform, adjoint, phi, jacobian, rotation);

      long fewestBytes = Long.MAX_VALUE;
      for (int round = 0; round < ALLOCATION_ROUNDS; round++)
      {
         long before = threadBean.getThreadAllocatedBytes(threadId);
         for (int i = 0; i < ALLOCATION_ITERATIONS; i++)
            sink += driveOnce(i, xiIn, xiOut, transform, adjoint, phi, jacobian, rotation);
         long after = threadBean.getThreadAllocatedBytes(threadId);

         fewestBytes = Math.min(fewestBytes, after - before);
      }

      // Keep the work observable so the loop cannot be optimized away wholesale.
      assertFalse(Double.isNaN(sink), "loop result went NaN");

      assertTrue(fewestBytes <= ALLOCATION_TOLERANCE_BYTES,
                 "exp/log/adjoint allocation-free overloads allocated " + fewestBytes + " bytes over " + ALLOCATION_ITERATIONS
                       + " iterations (tolerance " + ALLOCATION_TOLERANCE_BYTES + " B) — a per-tick allocation has regressed");
   }

   /** One exp → log → adjoint cycle through the allocation-free overloads. Must not allocate. */
   private static double driveOnce(int i,
                                   double[] xiIn,
                                   double[] xiOut,
                                   RigidBodyTransform transform,
                                   DMatrixRMaj adjoint,
                                   Vector3D phi,
                                   Matrix3D jacobian,
                                   RotationMatrix rotation)
   {
      // Vary the input without allocating, keeping |φ| well inside (0, π) so the generic (non-small-angle)
      // branch of the Jacobians is the one under test.
      double t = 1.0e-3 * (i % 1000);
      xiIn[0] = 0.3 + t;
      xiIn[1] = -0.2 + t;
      xiIn[2] = 0.5 - t;
      xiIn[3] = 1.0 + t;
      xiIn[4] = -2.0 + t;
      xiIn[5] = 0.7 + t;

      SE3LieGroupTools.exp(xiIn, transform, phi, jacobian);
      SE3LieGroupTools.log(transform, xiOut, phi, jacobian);
      SE3LieGroupTools.adjoint(transform, adjoint, rotation);

      return xiOut[0] + xiOut[5] + adjoint.unsafe_get(3, 0);
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
