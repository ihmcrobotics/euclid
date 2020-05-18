package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Pose3DTest extends Pose3DBasicsTest<Pose3D>
{
   private static final double EPSILON = 1e-7;

   @Override
   public Pose3D createEmptyPose3D()
   {
      return new Pose3D();
   }

   @Override
   public Pose3D createRandomPose3D(Random random)
   {
      return EuclidGeometryRandomTools.nextPose3D(random);
   }

   @Test
   public void testConstructors()
   {
      Random random = new Random(52942L);
      double x, y, z;
      Quaternion quaternion;
      Pose3D allDoubles, fromComponents, copiedPose, fromRBT;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         allDoubles = new Pose3D(x, y, z, quaternion.getYaw(), quaternion.getPitch(), quaternion.getRoll());

         assertEquals(x, allDoubles.getX(), EPSILON);
         assertEquals(y, allDoubles.getY(), EPSILON);
         assertEquals(z, allDoubles.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), allDoubles.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), allDoubles.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), allDoubles.getRoll(), EPSILON);

         fromComponents = new Pose3D(new Point3D(x, y, z), quaternion);

         assertEquals(x, fromComponents.getX(), EPSILON);
         assertEquals(y, fromComponents.getY(), EPSILON);
         assertEquals(z, fromComponents.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), fromComponents.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), fromComponents.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), fromComponents.getRoll(), EPSILON);

         copiedPose = new Pose3D(fromComponents);

         assertEquals(x, fromComponents.getX(), EPSILON);
         assertEquals(y, fromComponents.getY(), EPSILON);
         assertEquals(z, fromComponents.getZ(), EPSILON);
         assertEquals(fromComponents.getYaw(), copiedPose.getYaw(), EPSILON);
         assertEquals(fromComponents.getPitch(), copiedPose.getPitch(), EPSILON);
         assertEquals(fromComponents.getRoll(), copiedPose.getRoll(), EPSILON);

         fromRBT = new Pose3D(new RigidBodyTransform(new QuaternionBasedTransform(quaternion, new Point3D(x, y, z))));

         assertEquals(x, fromRBT.getX(), EPSILON);
         assertEquals(y, fromRBT.getY(), EPSILON);
         assertEquals(z, fromRBT.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), fromRBT.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), fromRBT.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), fromRBT.getRoll(), EPSILON);
      }
   }
}
