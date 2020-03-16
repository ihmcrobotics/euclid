package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.FrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools.GenericTypeBuilder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public final class FrameQuaternionTest extends FrameQuaternionReadOnlyTest<FrameQuaternion>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1e-10;

   @Override
   public FrameQuaternion createFrameTuple(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      if (tuple4DReadOnly instanceof QuaternionReadOnly)
         return new FrameQuaternion(referenceFrame, (QuaternionReadOnly) tuple4DReadOnly);
      else
         return new FrameQuaternion(referenceFrame, tuple4DReadOnly);
   }

   @Override
   public FrameQuaternion createFrameTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      Quaternion quaternion = new Quaternion();
      quaternion.setUnsafe(x, y, z, s);
      return new FrameQuaternion(referenceFrame, quaternion);
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameQuaternion()
         FrameQuaternion frameQuaternion = new FrameQuaternion();
         assertTrue(frameQuaternion.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(frameQuaternion);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertQuaternionIsSetToZero(frameQuaternion);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame,
                                                               randomQuaternion.getX(),
                                                               randomQuaternion.getY(),
                                                               randomQuaternion.getZ(),
                                                               randomQuaternion.getS());
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, double[] quaternionArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         double[] array = new double[4];
         randomQuaternion.get(array);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, array);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Quaternion randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(4, 1);
         randomQuaternion.get(denseMatrix);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, denseMatrix);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         QuaternionReadOnly randomQuaternion = EuclidCoreRandomTools.nextQuaternion(random);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomQuaternion);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Tuple4DReadOnly randomTuple = EuclidCoreRandomTools.nextVector4D(random);
         Quaternion expectedQuaternion = new Quaternion(randomTuple);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomTuple);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         RotationMatrixReadOnly randomRotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationMatrix);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomRotationMatrix);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         AxisAngleReadOnly randomAxisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Quaternion expectedQuaternion = new Quaternion(randomAxisAngle);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomAxisAngle);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3DReadOnly randomRotationVector = EuclidCoreRandomTools.nextVector3D(random);
         Quaternion expectedQuaternion = new Quaternion(randomRotationVector);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, randomRotationVector);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         Quaternion expectedQuaternion = new Quaternion(yaw, pitch, roll);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrame, yaw, pitch, roll);
         EuclidCoreTestTools.assertTuple4DEquals(expectedQuaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(FrameTuple4DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameTuple4DReadOnly randomFrameTuple4D = EuclidFrameRandomTools.nextFrameQuaternion(random, randomFrame);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrameTuple4D);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomFrameTuple4D, frameQuaternion, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(randomFrameTuple4D, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameQuaternion(FrameQuaternionReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion randomFrameQuaternion = EuclidFrameRandomTools.nextFrameQuaternion(random, randomFrame);
         FrameQuaternion frameQuaternion = new FrameQuaternion(randomFrameQuaternion);
         assertTrue(frameQuaternion.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple4DEquals(randomFrameQuaternion, frameQuaternion, EPSILON);
         EuclidFrameTestTools.assertFrameTuple4DEquals(randomFrameQuaternion, frameQuaternion, EPSILON);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         Quaternion expectedGeometryObject = EuclidCoreRandomTools.nextQuaternion(random);
         expectedGeometryObject.setToZero();

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         FrameQuaternion frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero();
         EuclidCoreTestTools.assertTuple4DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(expectedGeometryObject.epsilonEquals(frameGeometryObject, EPSILON));
         frameGeometryObject.setToZero(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(expectedGeometryObject, frameGeometryObject, EPSILON);
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(574);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         FrameQuaternion frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN();
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);

         frameGeometryObject = createRandomFrameTuple(random, initialFrame);
         ReferenceFrame newFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         assertEquals(initialFrame, frameGeometryObject.getReferenceFrame());
         assertFalse(frameGeometryObject.containsNaN());
         frameGeometryObject.setToNaN(newFrame);
         assertEquals(newFrame, frameGeometryObject.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(frameGeometryObject);
      }
   }

   @Test
   public void testMatchingFrame() throws Exception
   {
      Random random = new Random(3225);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameQuaternionReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameQuaternion expected = EuclidFrameRandomTools.nextFrameQuaternion(random, sourceFrame);
         FrameQuaternion actual = EuclidFrameRandomTools.nextFrameQuaternion(random, destinationFrame);

         actual.setMatchingFrame(expected);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple4DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple4DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameTuple4DBasics source = EuclidFrameRandomTools.nextFrameQuaternion(random, sourceFrame);
         FrameQuaternion actual = EuclidFrameRandomTools.nextFrameQuaternion(random, destinationFrame);

         actual.setMatchingFrame(source);
         FrameQuaternion expected = new FrameQuaternion(source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameTuple4DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         AxisAngleReadOnly axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setIncludingFrame(newFrame, axisAngle);
         quaternion.set(axisAngle);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         RotationMatrixReadOnly rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setIncludingFrame(newFrame, rotationMatrix);
         quaternion.set(rotationMatrix);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         Vector3DReadOnly rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setRotationVectorIncludingFrame(newFrame, rotationVector);
         quaternion.setRotationVector(rotationVector);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(FrameVector3DReadOnly rotationVector)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3DReadOnly rotationVector = new FrameVector3D(newFrame, EuclidCoreRandomTools.nextRotationVector(random));
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setRotationVectorIncludingFrame(rotationVector);
         quaternion.setRotationVector(rotationVector);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double[] yawPitchRoll)
         double[] yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRollArray(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setYawPitchRollIncludingFrame(newFrame, yawPitchRoll);
         quaternion.setYawPitchRoll(yawPitchRoll);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setYawPitchRollIncludingFrame(newFrame, yaw, pitch, roll);
         quaternion.setYawPitchRoll(yaw, pitch, roll);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly eulerAngles)
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         eulerAngles.setY(EuclidCoreTools.clamp(eulerAngles.getY(), YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE));
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setEulerIncludingFrame(newFrame, eulerAngles);
         quaternion.setEuler(eulerAngles);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, double rotX, double rotY, double rotZ)
         double rotX = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double rotY = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double rotZ = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameQuaternion frameQuaternion = createRandomFrameTuple(random, initialFrame);
         Quaternion quaternion = new Quaternion();
         assertEquals(initialFrame, frameQuaternion.getReferenceFrame());
         frameQuaternion.setEulerIncludingFrame(newFrame, rotX, rotY, rotZ);
         quaternion.setEuler(rotX, rotY, rotZ);
         assertEquals(newFrame, frameQuaternion.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, frameQuaternion, EPSILON);
      }
   }

   @Test
   public void testConsistencyWithQuaternion()
   {
      FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder = (frame, quaternion) -> createFrameTuple(frame, (QuaternionReadOnly) quaternion);
      GenericTypeBuilder framelessTypeBuilder = EuclidCoreRandomTools::nextQuaternion;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, framelessTypeBuilder, methodFilter);

      GenericTypeBuilder frameless2DTypeBuilder = (random) -> new Quaternion(createRandom2DFrameTuple(random, ReferenceFrame.getWorldFrame()));
      EuclidFrameAPITestTools.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder, frameless2DTypeBuilder, methodFilter);
   }

   @Override
   @Test
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("set", new Class<?>[] {Quaternion.class});
      framelessMethodsToIgnore.put("epsilonEquals", new Class<?>[] {Quaternion.class, Double.TYPE});
      framelessMethodsToIgnore.put("geometricallyEquals", new Class<?>[] {Quaternion.class, Double.TYPE});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameQuaternion.class, Quaternion.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Quaternion expected = EuclidCoreRandomTools.nextQuaternion(random);
         FrameQuaternion quaternion = new FrameQuaternion(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         quaternion.changeFrame(anotherFrame);
         assertTrue(anotherFrame == quaternion.getReferenceFrame());
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expected, quaternion, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            quaternion.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3452);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests set(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         QuaternionBasics expected = EuclidCoreRandomTools.nextQuaternion(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         FrameQuaternion actual = createRandomFrameTuple(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(EuclidCoreRandomTools.nextQuaternion(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, EuclidCoreRandomTools.nextQuaternion(random));
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testSetFromReferenceFrame() throws Exception
   {
      Random random = new Random(6572);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         FrameQuaternion expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         FrameQuaternion actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameQuaternion frameQuaternion1 = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameQuaternion frameQuaternion2 = new FrameQuaternion(worldFrame);
         double epsilon = random.nextDouble();

         AxisAngle axisAngleDiff;
         Quaternion difference;

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 0.99 * epsilon);
         difference = new Quaternion(axisAngleDiff);
         frameQuaternion2.multiply(frameQuaternion1, difference);
         assertTrue(frameQuaternion1.geometricallyEquals(frameQuaternion2, epsilon));

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 1.01 * epsilon);
         difference = new Quaternion(axisAngleDiff);
         frameQuaternion2.multiply(frameQuaternion1, difference);
         assertFalse(frameQuaternion1.geometricallyEquals(frameQuaternion2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FrameQuaternion q = EuclidFrameRandomTools.nextFrameQuaternion(random, frames[random.nextInt(frames.length)]);

      int newHashCode, previousHashCode;
      newHashCode = q.hashCode();
      assertEquals(newHashCode, q.hashCode());

      previousHashCode = q.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double qx = q.getX();
         double qy = q.getY();
         double qz = q.getZ();
         double qs = q.getS();
         switch (random.nextInt(4))
         {
            case 0:
               qx = random.nextDouble();
               break;
            case 1:
               qy = random.nextDouble();
               break;
            case 2:
               qz = random.nextDouble();
               break;
            case 3:
               qs = random.nextDouble();
               break;
         }
         q.setUnsafe(qx, qy, qz, qs);
         newHashCode = q.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;

         ReferenceFrame oldFrame = q.getReferenceFrame();
         ReferenceFrame newFrame = frames[random.nextInt(frames.length)];
         q.setReferenceFrame(newFrame);
         newHashCode = q.hashCode();
         if (oldFrame != newFrame)
            assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testQuaternionBasicsFeatures() throws Exception
   {
      QuaternionBasicsTest<FrameQuaternion> quaternionBasicsTest = new QuaternionBasicsTest<FrameQuaternion>()
      {
         @Override
         public FrameQuaternion createEmptyTuple()
         {
            return new FrameQuaternion();
         }

         @Override
         public FrameQuaternion createTuple(double v, double v1, double v2, double v3)
         {
            FrameQuaternion ret = new FrameQuaternion(ReferenceFrame.getWorldFrame());
            ret.setUnsafe(v, v1, v2, v3);
            return ret;
         }

         @Override
         public FrameQuaternion createRandomTuple(Random random)
         {
            return EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         }

         @Override
         public double getEpsilon()
         {
            return 1e-10;
         }
      };

      for (Method testMethod : quaternionBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(quaternionBasicsTest);
      }
   }
}