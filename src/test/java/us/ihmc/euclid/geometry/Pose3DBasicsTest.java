package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.RigidBodyTransformBasicsTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public abstract class Pose3DBasicsTest<T extends Pose3DBasics> extends RigidBodyTransformBasicsTest<T>
{
   private static final double EPSILON = 1e-7;

   public abstract T createEmptyPose3D();

   public abstract T createRandomPose3D(Random random);

   @Override
   public T createRandomTransform(Random random)
   {
      return createRandomPose3D(random);
   }

   public T copy(Pose3DReadOnly source)
   {
      T copy = createEmptyPose3D();
      copy.set(source);
      return copy;
   }

   @Override
   @Test
   public void testSetToNaN()
   {
      Random random = new Random(70324L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         T toSet = createRandomPose3D(random);

         toSet.setToNaN();

         assertEquals(Double.NaN, toSet.getX(), EPSILON);
         assertEquals(Double.NaN, toSet.getY(), EPSILON);
         assertEquals(Double.NaN, toSet.getZ(), EPSILON);
         assertEquals(Double.NaN, toSet.getYaw(), EPSILON);
         assertEquals(Double.NaN, toSet.getPitch(), EPSILON);
         assertEquals(Double.NaN, toSet.getRoll(), EPSILON);
      }
   }

   @Override
   @Test
   public void testSetToZero()
   {
      Random random = new Random(70924L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         T toSet = createRandomPose3D(random);

         toSet.setToZero();

         assertEquals(0, toSet.getX(), EPSILON);
         assertEquals(0, toSet.getY(), EPSILON);
         assertEquals(0, toSet.getZ(), EPSILON);
         assertEquals(0, toSet.getYaw(), EPSILON);
         assertEquals(0, toSet.getPitch(), EPSILON);
         assertEquals(0, toSet.getRoll(), EPSILON);
      }
   }

   @Test
   public void testSetComponents()
   {
      Random random = new Random(71484L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         double x = random.nextDouble() - random.nextDouble();
         double y = random.nextDouble() - random.nextDouble();
         double z = random.nextDouble() - random.nextDouble();

         T toSet = createRandomPose3D(random);
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         toSet.setX(x);
         toSet.setY(y);
         toSet.setZ(z);
         toSet.getOrientation().set(quaternion);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = createRandomPose3D(random);

         AxisAngle aa = EuclidCoreRandomTools.nextAxisAngle(random);

         toSet.getPosition().set(x, y, z);
         toSet.getOrientation().set(aa);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(aa.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(aa.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(aa.getRoll(), toSet.getRoll(), EPSILON);

         toSet = createRandomPose3D(random);
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         toSet.set(x, y, z, quaternion.getYaw(), quaternion.getPitch(), quaternion.getRoll());

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = createRandomPose3D(random);

         Vector3D tuple = new Vector3D(x, y, z);
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         toSet.set(tuple, quaternion);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = createRandomPose3D(random);
         RotationMatrix rot = EuclidCoreRandomTools.nextRotationMatrix(random);

         toSet.getPosition().set(tuple);
         toSet.getOrientation().set(rot);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(rot.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(rot.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(rot.getRoll(), toSet.getRoll(), EPSILON);

         toSet = createRandomPose3D(random);

         YawPitchRoll ypr = EuclidCoreRandomTools.nextYawPitchRoll(random);
         T toCopy = createEmptyPose3D();
         toCopy.getPosition().set(x, y, z);
         toCopy.getOrientation().set(ypr);

         assertEquals(ypr.getYaw(), toCopy.getYaw(), EPSILON);
         assertEquals(ypr.getPitch(), toCopy.getPitch(), EPSILON);
         assertEquals(ypr.getRoll(), toCopy.getRoll(), EPSILON);

         toSet.set(toCopy);

         assertEquals(x, toSet.getX(), EPSILON);
         assertEquals(y, toSet.getY(), EPSILON);
         assertEquals(z, toSet.getZ(), EPSILON);
         assertEquals(ypr.getYaw(), toSet.getYaw(), EPSILON);
         assertEquals(ypr.getPitch(), toSet.getPitch(), EPSILON);
         assertEquals(ypr.getRoll(), toSet.getRoll(), EPSILON);
      }
   }

   // TODO
   /*
    * @Test public void testOrientationDistance() { }
    */

   @Test
   public void testEquals()
   {
      Random random = new Random(9827L);
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double z = random.nextDouble() - random.nextDouble();
      Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
      Vector3D translation, orthogonal, normal = new Vector3D();

      T firstPose = createEmptyPose3D();
      firstPose.set(new Point3D(x, y, z), quaternion);
      T secondPose = copy(firstPose);

      // Sanity checks
      assertTrue(firstPose.equals(secondPose));
      assertTrue(secondPose.equals(secondPose));
      assertTrue(firstPose.equals(firstPose));
      assertTrue(secondPose.equals(secondPose));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if point components are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose.set(new Point3D(x, y, z), quaternion);
         secondPose.set(firstPose);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EPSILON);
         secondPose.appendTranslation(translation);

         assertFalse(firstPose.equals(secondPose));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if orientation angles are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose.set(new Point3D(x, y, z), quaternion);

         quaternion.getRotationVector(normal);
         orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         normal.applyTransform(new RigidBodyTransform(new AxisAngle(orthogonal, EPSILON), new Vector3D()));

         secondPose.set(x, y, z, normal.angle(Axis3D.Z), normal.angle(Axis3D.Y), normal.angle(Axis3D.X));

         assertFalse(firstPose.equals(secondPose));
      }
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(9827L);
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double z = random.nextDouble() - random.nextDouble();
      double qx, qy, qz, qs;
      Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);

      T firstPose = createEmptyPose3D();
      firstPose.set(new Point3D(x, y, z), quaternion);
      T secondPose = copy(firstPose);

      // Sanity checks
      assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(firstPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(secondPose, EPSILON));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when distance between point components is <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose.set(new Point3D(x, y, z), quaternion);
         secondPose.set(firstPose);

         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose.set(firstPose);

         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose.set(firstPose);

         secondPose.setZ(z + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose.set(firstPose);
         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose.set(firstPose);
         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose.set(firstPose);
         secondPose.setZ(z + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when difference between components of quaternions are each <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         qx = quaternion.getX();
         qy = quaternion.getY();
         qz = quaternion.getZ();
         qs = quaternion.getS();

         firstPose.set(new Point3D(x, y, z), quaternion);

         quaternion.setUnsafe(qx + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qy, qz, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qz, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz, qs + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qy, qz, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qz, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qs);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz, qs + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);
         secondPose.set(new Point3D(x, y, z), quaternion);
         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }
   }
}
