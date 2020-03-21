package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.api.*;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
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
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.euclid.yawPitchRoll.YawPitchRollBasicsTest;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public class FrameYawPitchRollTest
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FrameYawPitchRoll()
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll();
         assertTrue(frameYawPitchRoll.getReferenceFrame() == worldFrame);
         assertTrue(frameYawPitchRoll.getYaw() == 0.0);
         assertTrue(frameYawPitchRoll.getPitch() == 0.0);
         assertTrue(frameYawPitchRoll.getRoll() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame);
         assertTrue(frameYawPitchRoll.getReferenceFrame() == randomFrame);
         assertTrue(frameYawPitchRoll.getYaw() == 0.0);
         assertTrue(frameYawPitchRoll.getPitch() == 0.0);
         assertTrue(frameYawPitchRoll.getRoll() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, double[] yawPitchRollArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YawPitchRoll randomYawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         double[] array = new double[4];
         randomYawPitchRoll.get(array);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, array);
         assertTrue(frameYawPitchRoll.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertYawPitchRollEquals(randomYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, YawPitchRollReadOnly yawPitchRollReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         YawPitchRollReadOnly randomYawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, randomYawPitchRoll);
         assertTrue(frameYawPitchRoll.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertYawPitchRollEquals(randomYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         RotationMatrixReadOnly randomRotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         YawPitchRoll expectedYawPitchRoll = new YawPitchRoll(randomRotationMatrix);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, randomRotationMatrix);
         EuclidCoreTestTools.assertYawPitchRollEquals(expectedYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         AxisAngleReadOnly randomAxisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         YawPitchRoll expectedYawPitchRoll = new YawPitchRoll(randomAxisAngle);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, randomAxisAngle);
         EuclidCoreTestTools.assertYawPitchRollEquals(expectedYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3DReadOnly randomRotationVector = EuclidCoreRandomTools.nextVector3D(random);
         YawPitchRoll expectedYawPitchRoll = new YawPitchRoll(randomRotationVector);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, randomRotationVector);
         EuclidCoreTestTools.assertYawPitchRollEquals(expectedYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         YawPitchRoll expectedYawPitchRoll = new YawPitchRoll(yaw, pitch, roll);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrame, yaw, pitch, roll);
         EuclidCoreTestTools.assertYawPitchRollEquals(expectedYawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FrameYawPitchRoll(FrameYawPitchRollReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll randomFrameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, randomFrame);
         FrameYawPitchRoll frameYawPitchRoll = new FrameYawPitchRoll(randomFrameYawPitchRoll);
         assertTrue(frameYawPitchRoll.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertYawPitchRollEquals(randomFrameYawPitchRoll, frameYawPitchRoll, EPSILON);
      }
   }

   @Test
   public void testMatchingFrame() throws Exception
   {
      EuclidFrameAPITester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameYawPitchRoll,
                                                                       EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(3225);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameYawPitchRollReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         FrameYawPitchRoll expected = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, sourceFrame);
         FrameYawPitchRoll actual = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, destinationFrame);

         actual.setMatchingFrame(expected);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertFrameYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("setIncludingFrame", FrameVector3DReadOnly.class));
      signaturesToIgnore.add(new MethodSignature("setIncludingFrame", ReferenceFrame.class, Vector3DReadOnly.class));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);
      EuclidFrameAPITester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFrameYawPitchRoll,
                                                                        methodFilter,
                                                                        EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
         Orientation3DReadOnly orientation = EuclidCoreRandomTools.nextAxisAngle(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setIncludingFrame(newFrame, orientation);
         yawPitchRoll.set(orientation);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setRotationVectorIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
         Vector3DReadOnly rotationVector = EuclidCoreRandomTools.nextRotationVector(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setRotationVectorIncludingFrame(newFrame, rotationVector);
         yawPitchRoll.setRotationVector(rotationVector);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setRotationVectorIncludingFrame(FrameVector3DReadOnly rotationVector)
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3DReadOnly rotationVector = new FrameVector3D(newFrame, EuclidCoreRandomTools.nextRotationVector(random));
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setRotationVectorIncludingFrame(rotationVector);
         yawPitchRoll.setRotationVector(rotationVector);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double roll = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setYawPitchRollIncludingFrame(newFrame, yaw, pitch, roll);
         yawPitchRoll.setYawPitchRoll(yaw, pitch, roll);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly eulerAngles)
         Vector3D eulerAngles = EuclidCoreRandomTools.nextRotationVector(random);
         eulerAngles.setY(EuclidCoreTools.clamp(eulerAngles.getY(), YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE));
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setEulerIncludingFrame(newFrame, eulerAngles);
         yawPitchRoll.setEuler(eulerAngles);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests setEulerIncludingFrame(ReferenceFrame referenceFrame, double rotX, double rotY, double rotZ)
         double rotX = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double rotY = EuclidCoreRandomTools.nextDouble(random, YawPitchRollConversion.MAX_SAFE_PITCH_ANGLE);
         double rotZ = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameYawPitchRoll frameYawPitchRoll = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         assertEquals(initialFrame, frameYawPitchRoll.getReferenceFrame());
         frameYawPitchRoll.setEulerIncludingFrame(newFrame, rotX, rotY, rotZ);
         yawPitchRoll.setEuler(rotX, rotY, rotZ);
         assertEquals(newFrame, frameYawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(yawPitchRoll, frameYawPitchRoll, EPSILON);
      }
   }

   @Test
   public void testConsistencyWithYawPitchRoll()
   {
      FrameTypeCopier frameTypeBuilder = (frame, yawPitchRoll) -> new FrameYawPitchRoll(frame, (YawPitchRollReadOnly) yawPitchRoll);
      RandomFramelessTypeBuilder framelessTypeBuilder = EuclidCoreRandomTools::nextYawPitchRoll;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode");
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                framelessTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      RandomFramelessTypeBuilder frameless2DTypeBuilder = (random) -> new YawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0.0, 0.0);
      EuclidFrameAPITester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                                frameless2DTypeBuilder,
                                                                                methodFilter,
                                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().contains("MatchingFrame") && !m.getName().equals("equals")
            && !m.getName().equals("epsilonEquals");
      RandomFrameTypeBuilder frameTypeBuilder = EuclidFrameRandomTools::nextFrameYawPitchRoll;
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
      RandomFrameTypeBuilder frame2DTypeBuilder = (random, frame) -> new FrameYawPitchRoll(frame, nextDouble(random, Math.PI), 0.0, 0.0);
      EuclidFrameAPITester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame2DTypeBuilder,
                                                                                  methodFilter,
                                                                                  EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameOrientation3DReadOnly.class, Orientation3DReadOnly.class, true);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameOrientation3DBasics.class, Orientation3DBasics.class, true);
      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameYawPitchRollReadOnly.class, YawPitchRollReadOnly.class, true);

      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", YawPitchRoll.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", YawPitchRoll.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", YawPitchRoll.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester.assertOverloadingWithFrameObjects(FrameYawPitchRoll.class, YawPitchRoll.class, true, 1, methodFilter);
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

         YawPitchRoll expected = EuclidCoreRandomTools.nextYawPitchRoll(random);
         FrameYawPitchRoll yawPitchRoll = new FrameYawPitchRoll(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         yawPitchRoll.changeFrame(anotherFrame);
         assertTrue(anotherFrame == yawPitchRoll.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollGeometricallyEquals(expected, yawPitchRoll, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            yawPitchRoll.changeFrame(differentRootFrame);
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

         YawPitchRollBasics expected = EuclidCoreRandomTools.nextYawPitchRoll(random);

         int initialFrameIndex = random.nextInt(referenceFrames.length);
         ReferenceFrame initialFrame = referenceFrames[initialFrameIndex];
         FrameYawPitchRoll actual = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         actual.set(initialFrame, expected);

         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
         assertEquals(initialFrame, actual.getReferenceFrame());

         actual.set(EuclidCoreRandomTools.nextYawPitchRoll(random));

         assertFalse(expected.epsilonEquals(actual, EPSILON));

         expected.set(actual);

         int differenceFrameIndex = initialFrameIndex + random.nextInt(referenceFrames.length - 1) + 1;
         differenceFrameIndex %= referenceFrames.length;
         ReferenceFrame differentFrame = referenceFrames[differenceFrameIndex];

         try
         {
            actual.set(differentFrame, EuclidCoreRandomTools.nextYawPitchRoll(random));
            fail("Should have thrown a ReferenceFrameMismatchException");
         }
         catch (ReferenceFrameMismatchException e)
         {
            // good
            EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
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

         FrameYawPitchRoll expected = new FrameYawPitchRoll(anotherFrame);
         expected.changeFrame(initialFrame);

         FrameYawPitchRoll actual = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertYawPitchRollEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FrameYawPitchRoll frameYawPitchRoll1 = EuclidFrameRandomTools.nextFrameYawPitchRoll(random, worldFrame);
         FrameYawPitchRoll frameYawPitchRoll2 = new FrameYawPitchRoll(worldFrame);
         double epsilon = random.nextDouble();

         AxisAngle axisAngleDiff;
         YawPitchRoll difference;

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 0.99 * epsilon);
         difference = new YawPitchRoll(axisAngleDiff);
         frameYawPitchRoll2.set(frameYawPitchRoll1);
         frameYawPitchRoll2.append(difference);
         assertTrue(frameYawPitchRoll1.geometricallyEquals(frameYawPitchRoll2, epsilon));

         axisAngleDiff = new AxisAngle(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0), 1.01 * epsilon);
         difference = new YawPitchRoll(axisAngleDiff);
         frameYawPitchRoll2.set(frameYawPitchRoll1);
         frameYawPitchRoll2.append(difference);
         assertFalse(frameYawPitchRoll1.geometricallyEquals(frameYawPitchRoll2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(763);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll expected = EuclidCoreRandomTools.nextYawPitchRoll(random);
         FrameYawPitchRoll actual = new FrameYawPitchRoll(null, expected);

         assertEquals(expected.hashCode(), actual.hashCode());
      }
   }

   @Test
   public void testYawPitchRollBasicsFeatures() throws Exception
   {
      YawPitchRollBasicsTest<FrameYawPitchRoll> yawPitchRollBasicsTest = new YawPitchRollBasicsTest<FrameYawPitchRoll>()
      {
         @Override
         public FrameYawPitchRoll createEmptyYawPitchRoll()
         {
            return new FrameYawPitchRoll(worldFrame);
         }

         @Override
         public FrameYawPitchRoll createRandomYawPitchRoll(Random random)
         {
            return EuclidFrameRandomTools.nextFrameYawPitchRoll(random, worldFrame);
         }

         @Override
         public FrameYawPitchRoll createYawPitchRoll(double yaw, double pitch, double roll)
         {
            return new FrameYawPitchRoll(worldFrame, yaw, pitch, roll);
         }

         @Override
         public FrameYawPitchRoll createYawPitchRoll(Orientation3DReadOnly orientation3DReadOnly)
         {
            return new FrameYawPitchRoll(worldFrame, orientation3DReadOnly);
         }

         @Override
         public double getSmallestEpsilon()
         {
            return 1.0e-15;
         }

         @Override
         public double getEpsilon()
         {
            return 1e-10;
         }
      };

      for (Method testMethod : yawPitchRollBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(yawPitchRollBasicsTest);
      }
   }
}
