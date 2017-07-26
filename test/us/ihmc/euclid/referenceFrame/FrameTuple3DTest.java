package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.*;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Random;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Tuple3DBasicsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple3DTest<F extends FrameTuple3D<F, T>, T extends Tuple3DBasics & GeometryObject<T>> extends FrameTuple3DReadOnlyTest<F>
{
   public static final double epsilon = 1e-10;

   public final F createTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0);
   }

   public final F createTuple(ReferenceFrame referenceFrame, Tuple3DBasics tuple)
   {
      return createTuple(referenceFrame, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public final F createTuple(F frameTuple)
   {
      return createTuple(frameTuple.getReferenceFrame(), frameTuple);
   }

   public final F createRandomTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   protected ReferenceFrame theFrame = ReferenceFrame.constructARootFrame("theFrame");

   protected ReferenceFrame aFrame = ReferenceFrame.constructARootFrame("aFrame");

   protected RigidBodyTransform theFrameToChildFrame;

   protected ReferenceFrame childFrame;

   @Before
   public final void setUp() throws Exception
   {
      theFrameToChildFrame = new RigidBodyTransform();
      childFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", theFrame, theFrameToChildFrame);
   }

   @Test
   public final void testSetXYZ()
   {
      F frameTuple = createEmptyTuple();

      Random random = new Random(15613L);

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);

      double x = EuclidCoreRandomTools.generateRandomDouble(random, -100.0, 100.0);
      double y = EuclidCoreRandomTools.generateRandomDouble(random, -100.0, 100.0);
      double z = EuclidCoreRandomTools.generateRandomDouble(random, -100.0, 100.0);

      frameTuple.set(x, y, z);
      testGetters(frameTuple, x, y, z);

      frameTuple.setIncludingFrame(randomFrame, x, y, z);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      testGetters(frameTuple, x, y, z);
   }

   @Test //Brett was here
   public final void testSetTuple()
   {
      F frameTuple = createEmptyTuple();
      Random random = new Random(15613L);
      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);
      Tuple3DBasics randomTuple = EuclidCoreRandomTools.generateRandomPoint3D(random, 100.0, 100.0, 100.0);

      frameTuple.set(randomTuple);
      testGetters(frameTuple, randomTuple);

      frameTuple.setIncludingFrame(randomFrame, randomTuple);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      assertEquals(randomFrame, frameTuple.getReferenceFrame());
      testGetters(frameTuple, randomTuple);

      //test non-matching reference frames
      F ft1 = createTuple(aFrame);
      F ft2 = createTuple(theFrame);
      try
      {
         ft1.set(ft2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testSetFrameTuple()
   {
      F frameTuple = createEmptyTuple();
      F randomFrameTuple = createEmptyTuple();

      Random random = new Random(15613L);

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("childFrame", ReferenceFrame.getWorldFrame(), randomTransform);

      Tuple3DBasics randomTuple = EuclidCoreRandomTools.generateRandomPoint3D(random, 100.0, 100.0, 100.0);

      randomFrameTuple.setIncludingFrame(randomFrame, randomTuple);

      try
      {
         frameTuple.set(randomFrameTuple);
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      frameTuple.setToZero(randomFrame);
      assertEquals(randomFrame, frameTuple.getReferenceFrame());

      frameTuple.set(randomFrameTuple);
      EuclidFrameTestTools.assertFrameTuple3DEquals(frameTuple, randomFrameTuple, epsilon);

      frameTuple = createEmptyTuple();

      frameTuple.setIncludingFrame(randomFrameTuple);
      EuclidFrameTestTools.assertFrameTuple3DEquals(frameTuple, randomFrameTuple, epsilon);
   }

   private final void testGetters(F frameTuple, Tuple3DBasics tuple)
   {
      testGetters(frameTuple, tuple.getX(), tuple.getY(), tuple.getZ());
      assertTrue(frameTuple.epsilonEquals(tuple, epsilon));
   }

   private final void testGetters(F frameTuple, double x, double y, double z)
   {
      assertEquals(x, frameTuple.getX(), epsilon);
      assertEquals(y, frameTuple.getY(), epsilon);
      assertEquals(z, frameTuple.getZ(), epsilon);

      Tuple3DBasics tuple3dToTest = new Point3D();
      frameTuple.get(tuple3dToTest);
      assertEquals(x, tuple3dToTest.getX(), epsilon);
      assertEquals(y, tuple3dToTest.getY(), epsilon);
      assertEquals(z, tuple3dToTest.getZ(), epsilon);

      assertEquals(x, frameTuple.getX(), epsilon);
      assertEquals(y, frameTuple.getY(), epsilon);
      assertEquals(z, frameTuple.getZ(), epsilon);
   }

   // Tests copied from FramePointTest and FrameVectorTest

   @Test
   public final void testChangeFrame() throws Exception
   {
      Random random = new Random(1776L);

      F vWorld = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);

      int numberOfFrames = 10;
      ReferenceFrame[] frames = EuclidFrameRandomTools.generateRandomReferenceFrameTree(random, numberOfFrames);

      ArrayList<F> resultVectors = new ArrayList<F>();
      resultVectors.add(vWorld);

      // Choose random paths and move the vectors around those paths:
      int numVectors = 1000;

      for (int i = 0; i < numVectors; i++)
      {
         int pathLength = random.nextInt(20);

         F vector = createTuple(vWorld);

         for (int j = 0; j < pathLength; j++)
         {
            int frameIndex = random.nextInt(frames.length);
            vector.changeFrame(frames[frameIndex]);
         }

         vector.changeFrame(vWorld.getReferenceFrame());
         resultVectors.add(vector);
      }

      // Now compare all sets of 2 vectors. If they are in the same frame, they should have the same values
      for (F resultVector1 : resultVectors)
      {
         for (F resultVector2 : resultVectors)
         {
            EuclidFrameTestTools.assertFrameTuple3DEquals(resultVector1, resultVector2, epsilon);
         }
      }
   }

   @Test
   public final void testSets() throws Exception //Brett was here
   {
      F alpha = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      F beta = createTuple(ReferenceFrame.getWorldFrame(), 8.0, -2.0, 0.0);
      F ones = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

      alpha.set(1.0, 1.0, 1.0);
      assertTrue("This should be true", alpha.epsilonEquals(ones, 1e-10));

      alpha.setX(-7.0);
      assertEquals("This should be equal", -7.0, alpha.getX(), 1e-10);

      alpha.setY(10.3);
      assertEquals("This should be equal", 10.3, alpha.getY(), 1e-10);

      alpha.setZ(1.9);
      assertEquals("This should be equal", 1.9, alpha.getZ(), 1e-10);

      alpha.set(10, 20, 30);
      assertEquals("This should be equal", 10, alpha.getX(), 1e-10);
      assertEquals("This should be equal", 20, alpha.getY(), 1e-10);
      assertEquals("This should be equal", 30, alpha.getZ(), 1e-10);

      alpha.setX(0);
      assertEquals("This should be equal", 0, alpha.getX(), 1e-10);

      alpha.set(beta);
      assertTrue("This should be true", alpha.epsilonEquals(beta, 1e-10));
   }

   @Test //Brett was here
   public final void testSetXY()
   {
      F alpha = createTuple(theFrame, 1.0, 2.0, 3.0);
      FramePoint2D framepoint2d = new FramePoint2D(theFrame);
      framepoint2d.set(-1.0, -2.0);

      alpha.set(framepoint2d, 0.0);
      assertEquals("This should be equal", -1.0, alpha.getX(), epsilon);
      assertEquals("This should be equal", -2.0, alpha.getY(), epsilon);
      assertEquals("This should be equal", 0.0, alpha.getZ(), epsilon);

      //test non-matching reference frames
      F ft1 = createTuple(aFrame);
      try
      {
         ft1.set(framepoint2d);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testGets()
   {
      F alpha = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      F beta = createTuple(ReferenceFrame.getWorldFrame(), 7.0, 0.0, -6.0);
      alpha.getX();
      beta.getY();
      beta.getZ();
   }

   @Test
   public final void testAddTuple3d()
   {
      F alpha = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      F expected = createTuple(ReferenceFrame.getWorldFrame(), 2.0, 3.0, 4.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      alpha.add(tuple1);
      assertTrue(alpha.epsilonEquals(expected, epsilon));
   }

   @Test
   public final void testAddTuple3dTuple3d()
   {
      F alpha = createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      F expected = createTuple(ReferenceFrame.getWorldFrame(), 2.0, 2.0, 2.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      Point3D tuple2 = new Point3D(1.0, 1.0, 1.0);

      alpha.add(tuple1, tuple2);
      assertTrue(alpha.epsilonEquals(expected, epsilon));
   }

   @Test
   public final void testAddFrameTuple() //Brett
   {
      F frameTuple1 = createTuple(theFrame);
      F vector = createTuple(theFrame, 10.0, 11.0, 12.0);
      frameTuple1.add(vector);

      assertEquals(10.0, frameTuple1.getX(), epsilon);
      assertEquals(11.0, frameTuple1.getY(), epsilon);
      assertEquals(12.0, frameTuple1.getZ(), epsilon);

      //Test non-matching reference frames
      F frameTuple2 = createTuple(aFrame, 10.0, 11.0, 12.0);
      try
      {
         frameTuple1.add(frameTuple2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testAddFrameTupleFrameTuple() //Brett
   {
      F expected = createTuple(theFrame);
      F frameTuple1 = createTuple(theFrame, 0.1, 0.1, 0.1);
      F frameTuple2 = createTuple(theFrame, 10, 10, 10);

      expected.add(frameTuple1, frameTuple2);
      assertEquals(10.1, expected.getX(), epsilon);
      assertEquals(10.1, expected.getY(), epsilon);
      assertEquals(10.1, expected.getZ(), epsilon);

      F expected2 = createTuple(theFrame);
      F frameTuple2frame = createTuple(aFrame, 10, 10, 10);
      try
      {
         expected2.add(frameTuple1, frameTuple2frame);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         expected2.add(frameTuple2frame, frameTuple1);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @Test
   public final void testReferenceFramesAreCheckedOnSet()
   {
      F framePointOne = createTuple(theFrame);
      F framePointTwo = createTuple(aFrame);

      try
      {
         framePointOne.set(framePointTwo);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testEpsilonEqualsTuple()
   {
      F framePoint = createTuple(theFrame, 1.0, 2.0, 3.0);
      Point3D tuple1 = new Point3D(1.0, 2.0, 3.0);

      boolean tupleResult = tuple1.epsilonEquals(tuple1, epsilon);
      boolean framePointResult = framePoint.epsilonEquals(tuple1, epsilon);

      assertTrue(tupleResult == framePointResult);
   }

   @Test
   public final void testEpsilonEqualsFrameTuple()
   {
      F framePoint1 = createTuple(theFrame, 1.0, 2.0, 3.0);
      F framePoint2 = createTuple(aFrame, 1.0, 2.0, 3.0);

      assertFalse(framePoint1.epsilonEquals(framePoint2, epsilon)); //test with non-matching refernce frames

      double threshold = 0.5;
      boolean expectedReturn = true;

      F framePoint = createTuple(theFrame, 1.1, 2.1, 3.1);
      boolean actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createTuple(theFrame, 1.2, 2.2, 3.2);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      expectedReturn = false;
      framePoint = createTuple(theFrame, 1.7, 2.1, 3.1);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createTuple(theFrame, 1.1, 2.7, 3.1);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);

      framePoint = createTuple(theFrame, 0, 0, 0);
      actualReturn = framePoint1.epsilonEquals(framePoint, threshold);
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @Test
   public final void testGetTuple3d()
   {
      F frameTuple = createTuple(theFrame, 1.0, 2.0, 3.0);
      Vector3D tuple3d = new Vector3D();
      frameTuple.get(tuple3d);

      assertTrue(frameTuple.epsilonEquals(tuple3d, epsilon));
   }

   @Test
   public final void testSetToZero() throws Exception
   {
      F frameTuple = createTuple(theFrame, 1.0, 2.0, 3.0);
      F frameTupleZero = createTuple(theFrame, 0.0, 0.0, 0.0);
      frameTuple.setToZero();
      assertTrue(frameTuple.epsilonEquals(frameTupleZero, epsilon));

      frameTuple = createTuple(theFrame, 1.0, 2.0, 3.0);
      frameTupleZero = createTuple(aFrame, 0.0, 0.0, 0.0);
      frameTuple.setToZero(aFrame);
      assertTrue(frameTuple.epsilonEquals(frameTupleZero, epsilon));
   }

   @Test
   public final void testSetToNaN() throws Exception
   {
      F frameTuple = createTuple(theFrame, 1.0, 2.0, 3.0);
      frameTuple.setToNaN();
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
      assertTrue(Double.isNaN(frameTuple.getZ()));

      frameTuple = createTuple(theFrame, 1.0, 2.0, 3.0);
      frameTuple.setToNaN(aFrame);
      assertTrue(frameTuple.getReferenceFrame() == aFrame);
      assertTrue(Double.isNaN(frameTuple.getX()));
      assertTrue(Double.isNaN(frameTuple.getY()));
      assertTrue(Double.isNaN(frameTuple.getZ()));

      F frameVector = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector2 = createTuple(theFrame, 20.0, -7.0, -14.0);

      frameVector.setToNaN();
      assertTrue(Double.isNaN(frameVector.getX()));
      assertTrue(Double.isNaN(frameVector.getY()));
      assertTrue(Double.isNaN(frameVector.getZ()));

      frameVector2.setToNaN(theFrame);
      assertTrue(Double.isNaN(frameVector2.getX()));
      assertTrue(Double.isNaN(frameVector2.getY()));
      assertTrue(Double.isNaN(frameVector2.getZ()));
   }

   @Test
   public final void testGetReferenceFrame()
   {
      F framePoint = createTuple(theFrame);

      ReferenceFrame expectedReturn = theFrame;
      ReferenceFrame actualReturn = framePoint.getReferenceFrame();
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @Test
   public final void testGetX()
   {
      F framePoint = createTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.1;
      double actualReturn = framePoint.getX();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @Test
   public final void testGetY()
   {
      F framePoint = createTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.2;
      double actualReturn = framePoint.getY();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @Test
   public final void testGetZ()
   {
      F framePoint = createTuple(theFrame, 1.1, 1.2, 1.3);

      double expectedReturn = 1.3;
      double actualReturn = framePoint.getZ();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);
   }

   @Test
   public final void testSet2()
   {
      F framePoint = createTuple(theFrame);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      framePoint.set(x, y, z);

      assertEquals(x, framePoint.getX(), epsilon);
      assertEquals(y, framePoint.getY(), epsilon);
      assertEquals(z, framePoint.getZ(), epsilon);
   }

   @Test
   public final void testSetX()
   {
      F framePoint = createTuple(theFrame);

      double x = 0.0;
      framePoint.setX(x);

      assertEquals(x, framePoint.getX(), epsilon);
   }

   @Test
   public final void testSetY()
   {
      F framePoint = createTuple(theFrame);

      double y = 0.0;
      framePoint.setY(y);

      assertEquals(y, framePoint.getY(), epsilon);
   }

   @Test
   public final void testSetZ()
   {
      F framePoint = createTuple(theFrame);

      double z = 0.0;
      framePoint.setZ(z);

      assertEquals(z, framePoint.getZ(), epsilon);
   }

   @Test
   public final void testSubTuple3d()
   {
      F frameTuple1 = createTuple(theFrame, 1.0, 1.0, 1.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);

      frameTuple1.sub(tuple1);
      tuple1.sub(tuple1);

      assertTrue(frameTuple1.epsilonEquals(tuple1, epsilon));
   }

   @Test
   public final void testSubTuple3dTuple3d()
   {
      F frameTuple1 = createTuple(theFrame, 1.0, 1.0, 1.0);
      F frameTuple2 = createTuple(theFrame, 1.0, 1.0, 1.0);
      F expectedFrameTuple = createTuple(theFrame, 0.0, 0.0, 0.0);
      Point3D tuple1 = new Point3D(1.0, 1.0, 1.0);
      Point3D tuple2 = new Point3D(1.0, 1.0, 1.0);
      Point3D expectedTuple = new Point3D(0.0, 0.0, 0.0);

      expectedFrameTuple.sub(frameTuple1, frameTuple2);
      expectedTuple.sub(tuple1, tuple2);

      assertTrue(expectedFrameTuple.epsilonEquals(expectedTuple, epsilon));
   }

   @Test
   public final void testSubFrameTuple()
   {
      F framePoint1 = createTuple(theFrame);
      F framePoint = null;
      F framePoint2 = createTuple(aFrame);

      try
      {
         framePoint1.sub(framePoint);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint1.sub(framePoint2);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @Test
   public final void testSubFrameTupleFrameTuple()
   {
      F framePoint = createTuple(theFrame);
      F framePoint2 = createTuple(aFrame);
      F point1 = null;
      F point2 = null;
      try
      {
         framePoint.sub(point1, point2);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint.sub(point2, point1);
         fail("Should have thrown NullPointerException");
      }
      catch (NullPointerException npe)
      {
         //Good
      }
      try
      {
         framePoint.sub(framePoint, framePoint2);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         framePoint.sub(framePoint2, framePoint);
         fail("Should have thrown ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @Test
   public final void testCheckReferenceFrameMatch() throws ReferenceFrameMismatchException
   {
      F framePoint = createTuple(theFrame);

      ReferenceFrame frame = null;
      try
      {
         framePoint.checkReferenceFrameMatch(frame);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testFramePoint()
   {
      createTuple(theFrame);

      ReferenceFrame referenceFrame = null;
      createTuple(referenceFrame);
   }

   @Test
   public final void testFramePoint2()
   {
      createTuple(theFrame);

      ReferenceFrame referenceFrame = null;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      createTuple(referenceFrame, x, y, z);
   }

   @Test //Brett was here
   public final void testScale()
   {
      F framePoint = createTuple(theFrame, 5.0, 5.0, 5.0);
      F tuple1 = createTuple(aFrame, 5.0, 5.0, 5.0);

      double scaleFactor = 4.0;
      framePoint.scale(scaleFactor);

      double expectedReturn = 20.0;
      double actualReturn = framePoint.getZ();
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      framePoint.set(1.0, 2.0, 3.0);
      framePoint.scale(3.0, 3.0 / 2.0, 1.0);
      assertEquals("return value", 3.0, framePoint.getX(), Double.MIN_VALUE);
      assertEquals("return value", 3.0, framePoint.getY(), Double.MIN_VALUE);
      assertEquals("return value", 3.0, framePoint.getZ(), Double.MIN_VALUE);

      //test non-matching reference frames
      try
      {
         framePoint.setAndScale(scaleFactor, tuple1);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleVector()
   {
      F framePoint = createTuple(theFrame);
      F frameVector = createTuple(theFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, frameVector);

      assertEquals(2.3, framePoint.getX(), epsilon);
      assertEquals(6.9, framePoint.getY(), epsilon);
      assertEquals(-4.6, framePoint.getZ(), epsilon);
   }

   @Test
   public final void testScaleVectorException()
   {
      F framePoint = createTuple(theFrame);
      F frameVector = createTuple(aFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      try
      {
         framePoint.setAndScale(scale, frameVector);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScalePoint()
   {
      F framePoint = createTuple(theFrame);
      F framePoint2 = createTuple(theFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      framePoint.setAndScale(scale, framePoint2);

      assertEquals(2.3, framePoint.getX(), epsilon);
      assertEquals(6.9, framePoint.getY(), epsilon);
      assertEquals(-4.6, framePoint.getZ(), epsilon);
   }

   @Test
   public final void testScalePointException()
   {
      F framePoint = createTuple(theFrame);
      F framePoint2 = createTuple(aFrame, 1.0, 3.0, -2.0);
      double scale = 2.3;

      try
      {
         framePoint.setAndScale(scale, framePoint2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddVectorVector() //Brett was here
   {
      F framePoint = createTuple(theFrame);
      F frameVector1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePoint.scaleAdd(scale, frameVector1, frameVector2);

      assertEquals(scale * 7.0 + 1.0, framePoint.getX(), epsilon);
      assertEquals(scale * -1.5 + 3.0, framePoint.getY(), epsilon);
      assertEquals(scale * -2.0 + 3.6, framePoint.getZ(), epsilon);

      F frameVector3 = createTuple(aFrame, 1.0, 3.0, 3.6);
      try
      {
         framePoint.scaleAdd(scale, frameVector1, frameVector3);
         fail("Should throw ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
      try
      {
         framePoint.scaleAdd(scale, frameVector3, frameVector1);
         fail("Should throw ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException rfme)
      {
         //Good
      }
   }

   @Test
   public final void testScaleAddVectorVectorException1()
   {
      F framePoint = createTuple(theFrame);
      F frameVector1 = createTuple(aFrame, 7.0, -1.5, -2.0);
      F frameVector2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePoint.scaleAdd(scale, frameVector1, frameVector2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddVectorVectorException2()
   {
      F framePoint = createTuple(theFrame);
      F frameVector1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector2 = createTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePoint.scaleAdd(scale, frameVector1, frameVector2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddVectorPoint()
   {
      F framePointResult1 = createTuple(theFrame);
      F framePointResult2 = createTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      F frameVector1 = createTuple(theFrame, vector1);
      F framePoint1 = createTuple(theFrame, vector2);
      F frameVector2 = createTuple(theFrame, vector2);
      F framePoint2 = createTuple(theFrame, vector1);

      framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
      assertEquals(2.3 * 7.0 + 1.0, framePointResult1.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult1.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult1.getZ(), epsilon);

      framePointResult2.scaleAdd(scale, framePoint2, frameVector2);
      assertTrue(framePointResult1.epsilonEquals(framePointResult2, epsilon));
   }

   @Test
   public final void testScaleAddVectorPointException1()
   {
      F framePointResult1 = createTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      F frameVector1 = createTuple(aFrame, vector1);
      F framePoint1 = createTuple(theFrame, vector2);

      try
      {
         framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddVectorPointException2()
   {
      F framePointResult1 = createTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      F frameVector1 = createTuple(theFrame, vector1);
      F framePoint1 = createTuple(aFrame, vector2);

      try
      {
         framePointResult1.scaleAdd(scale, frameVector1, framePoint1);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddPointVectorException1()
   {
      F framePointResult1 = createTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      F frameVector1 = createTuple(aFrame, vector1);
      F framePoint1 = createTuple(theFrame, vector2);

      try
      {
         framePointResult1.scaleAdd(scale, framePoint1, frameVector1);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddPointVectorException2()
   {
      F framePointResult1 = createTuple(theFrame);

      Vector3D vector1 = new Vector3D(7.0, -1.5, -2.0);
      Vector3D vector2 = new Vector3D(1.0, 3.0, 3.6);
      double scale = 2.3;

      F frameVector1 = createTuple(theFrame, vector1);
      F framePoint1 = createTuple(aFrame, vector2);

      try
      {
         framePointResult1.scaleAdd(scale, framePoint1, frameVector1);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddPointPoint()
   {
      F framePointResult = createTuple(theFrame);
      F framePoint1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint1, framePoint2);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);
   }

   @Test
   public final void testScaleAddPointPointException1()
   {
      F framePointResult = createTuple(theFrame);
      F framePoint1 = createTuple(aFrame, 7.0, -1.5, -2.0);
      F framePoint2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePointResult.scaleAdd(scale, framePoint1, framePoint2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddPointPointException2()
   {
      F framePointResult = createTuple(theFrame);
      F framePoint1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint2 = createTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePointResult.scaleAdd(scale, framePoint1, framePoint2);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test //Brett was here
   public final void testScaleAddScaleTuple()
   {
      F framePointResult = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, frameVector);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);

      //test non-matching reference frames
      F ft1 = createTuple(aFrame);
      try
      {
         ft1.scaleAdd(scale, frameVector);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddVectorException()
   {
      F framePointResult = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector = createTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePointResult.scaleAdd(scale, frameVector);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testScaleAddPoint()
   {
      F framePointResult = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint = createTuple(theFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      framePointResult.scaleAdd(scale, framePoint);

      assertEquals(2.3 * 7.0 + 1.0, framePointResult.getX(), epsilon);
      assertEquals(2.3 * -1.5 + 3.0, framePointResult.getY(), epsilon);
      assertEquals(2.3 * -2.0 + 3.6, framePointResult.getZ(), epsilon);
   }

   @Test
   public final void testScaleAddPointException()
   {
      F framePointResult = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint = createTuple(aFrame, 1.0, 3.0, 3.6);
      double scale = 2.3;

      try
      {
         framePointResult.scaleAdd(scale, framePoint);
         fail("Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName());
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good.
      }
   }

   @Test
   public final void testSubFramePoint()
   {
      F framePoint1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      F expectedResult = createTuple(theFrame, 6.0, -4.5, -5.6);

      framePoint1.sub(framePoint2);
      assertTrue(expectedResult.epsilonEquals(framePoint1, epsilon));
   }

   @Test
   public final void testSubFrameVector()
   {
      F framePoint = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector = createTuple(theFrame, 1.0, 3.0, 3.6);
      F expectedResult = createTuple(theFrame, 6.0, -4.5, -5.6);

      framePoint.sub(frameVector);
      assertTrue(expectedResult.epsilonEquals(framePoint, epsilon));
   }

   @Test
   public final void testSubPointPoint()
   {
      F framePoint1 = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint2 = createTuple(theFrame, 1.0, 3.0, 3.6);
      F expectedResult = createTuple(theFrame, 6.0, -4.5, -5.6);

      F actualResult = createTuple(theFrame);
      actualResult.sub(framePoint1, framePoint2);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @Test
   public final void testSubPointVector()
   {
      F framePoint = createTuple(theFrame, 7.0, -1.5, -2.0);
      F frameVector = createTuple(theFrame, 1.0, 3.0, 3.6);
      F expectedResult = createTuple(theFrame, 6.0, -4.5, -5.6);

      F actualResult = createTuple(theFrame);
      actualResult.sub(framePoint, frameVector);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @Test
   public final void testSubVectorPoint()
   {
      F frameVector = createTuple(theFrame, 7.0, -1.5, -2.0);
      F framePoint = createTuple(theFrame, 1.0, 3.0, 3.6);
      F expectedResult = createTuple(theFrame, 6.0, -4.5, -5.6);

      F actualResult = createTuple(theFrame);
      actualResult.sub(frameVector, framePoint);
      assertTrue(expectedResult.epsilonEquals(actualResult, epsilon));
   }

   @Test
   public final void testInterpolate()
   {
      F frameTuple1 = createTuple(ReferenceFrame.getWorldFrame(), -1.0, 0.0, 17.0);
      F frameTuple2 = createTuple(ReferenceFrame.getWorldFrame(), 3.3, 30.0, 9.0);
      F frameTuple3 = createTuple(ReferenceFrame.getWorldFrame(), 1.0, 2.8, 3.0);

      frameTuple3.interpolate(frameTuple1, frameTuple2, 3.0);
      frameTuple3.interpolate(frameTuple2, frameTuple1, 1);

      F frameTuple4 = createTuple(theFrame, 1.0, 2.0, 3.0);
      F frameTuple5 = createTuple(theFrame, 0.0, -1.0, 8.2);
      double alpha = 0.57;

      F resultTuple = createTuple(frameTuple4.getReferenceFrame(), frameTuple4.getX(), frameTuple4.getY(), frameTuple4.getZ());
      resultTuple.interpolate(frameTuple4, frameTuple5, alpha);

      assertEquals((1.0 - alpha) * frameTuple4.getX() + alpha * frameTuple5.getX(), resultTuple.getX(), epsilon);
      assertEquals((1.0 - alpha) * frameTuple4.getY() + alpha * frameTuple5.getY(), resultTuple.getY(), epsilon);
      assertEquals((1.0 - alpha) * frameTuple4.getZ() + alpha * frameTuple5.getZ(), resultTuple.getZ(), epsilon);
   }

   @Test
   public final void testPackMatrix() //Brett was here
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      F frametuple = createTuple(worldFrame, 10.0, 10.0, 10.0);
      Random ran = new Random(4564L);
      int numberOfIterations = 100;
      int startRow;
      for (int counter = 0; counter < numberOfIterations; counter++)
      {
         int numRows = ran.nextInt(10) + 3; //3-13 exclusive
         int numCols = ran.nextInt(10) + 3;
         if (ran.nextInt(Math.max(numRows - 3, 1)) <= 0)
         {
            startRow = 0;
         }
         else
         {
            startRow = ran.nextInt(numRows - 3);
         }
         //         int startRow = ran.nextInt(numRows - 3); //3 fewer than number of last row because of 3-tuple x, y, z

         DenseMatrix64F matrix = new DenseMatrix64F(numRows, numCols);
         //      System.out.println(matrix.toString()); //before
         frametuple.get(startRow, matrix);
         //      System.out.println(matrix.toString()); //after

         for (int i = startRow; i < startRow + 3; i++)
         {
            //            System.out.println("Entry " + i + " " + matrix.get(i, 0));
            assertEquals("Should be equal", 10.0, matrix.get(i, 0), Double.MIN_VALUE);
         }
      }
   }

   @Test
   public final void testClipToMinMax() //Brett was here
   {
      F frameTuple = createTuple(ReferenceFrame.getWorldFrame(), -5.0, 3.0, 10.0);
      frameTuple.clipToMinMax(4, 10); //call clipToMinMax(4, 10)
      assertEquals("Should be equal", 4, frameTuple.getX(), epsilon);
      assertEquals("Should be equal", 4, frameTuple.getY(), epsilon);
      assertEquals("Should be equal", 10, frameTuple.getZ(), epsilon);
   }

   @Test
   public final void testNegate() //Brett was here
   {
      F frameTuple = createTuple(ReferenceFrame.getWorldFrame(), -5.0, 0.0, 10.0);
      F frameTupleToNegate = createTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, -10.0);
      frameTupleToNegate.negate();
      assertEquals("Should be equal", frameTuple.getX(), frameTupleToNegate.getX(), epsilon);
      assertEquals("Should be equal", frameTuple.getY(), frameTupleToNegate.getY(), epsilon);
      assertEquals("Should be equal", frameTuple.getZ(), frameTupleToNegate.getZ(), epsilon);
   }

   @Test
   public final void testAbsolute()
   {
      F expectedFrameTuple = createTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, 10.0);
      F actualFrameTuple = createTuple(ReferenceFrame.getWorldFrame(), 5.0, 0.0, -10.0);
      actualFrameTuple.absolute();
      assertEquals("Should be equal", expectedFrameTuple.getX(), actualFrameTuple.getX(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getY(), actualFrameTuple.getY(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getZ(), actualFrameTuple.getZ(), epsilon);

      expectedFrameTuple = createTuple(ReferenceFrame.getWorldFrame(), 5.0, 1.0, 10.0);
      actualFrameTuple = createTuple(ReferenceFrame.getWorldFrame(), 5.0, -1.0, -10.0);
      actualFrameTuple.absolute();
      assertEquals("Should be equal", expectedFrameTuple.getX(), actualFrameTuple.getX(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getY(), actualFrameTuple.getY(), epsilon);
      assertEquals("Should be equal", expectedFrameTuple.getZ(), actualFrameTuple.getZ(), epsilon);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      assertSuperMethodsAreOverloaded(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, FrameTuple3D.class, Tuple3DBasics.class);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().contains("IncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomTuple(random, frame), false, true, methodFilter);
   }

   @Test
   public void testFrameGeometryObjectFeatures() throws Throwable
   {
      FrameGeometryObjectTest<F, T> frameGeometryObjectTest = new FrameGeometryObjectTest<F, T>()
      {
         @Override
         public T createEmptyGeometryObject()
         {
            return createEmptyTuple().getGeometryObject();
         }

         @Override
         public T createRandomGeometryObject(Random random)
         {
            return createRandomTuple(random).getGeometryObject();
         }

         @Override
         public F createEmptyFrameGeometryObject(ReferenceFrame referenceFrame)
         {
            return createEmptyTuple(referenceFrame);
         }

         @Override
         public F createFrameGeometryObject(ReferenceFrame referenceFrame, T geometryObject)
         {
            return createTuple(referenceFrame, geometryObject);
         }

         @Override
         public F createRandomFrameGeometryObject(Random random, ReferenceFrame referenceFrame)
         {
            return createRandomTuple(random, referenceFrame);
         }
      };

      for (Method testMethod : frameGeometryObjectTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         try
         {
            testMethod.invoke(frameGeometryObjectTest);
         }
         catch (InvocationTargetException e)
         {
            throw e.getCause();
         }
      }
   }

   @Test
   public void testTuple3DBasicsFeatures() throws Exception
   {
      Tuple3DBasicsTest<F> tuple3dBasicsTest = new Tuple3DBasicsTest<F>()
      {
         @Override
         public F createEmptyTuple()
         {
            return FrameTuple3DTest.this.createEmptyTuple();
         }

         @Override
         public F createTuple(double x, double y, double z)
         {
            return FrameTuple3DTest.this.createTuple(x, y, z);
         }

         @Override
         public F createRandomTuple(Random random)
         {
            return FrameTuple3DTest.this.createRandomTuple(random);
         }

         @Override
         public double getEpsilon()
         {
            return FrameTuple3DTest.this.getEpsilon();
         }
      };

      for (Method testMethod : tuple3dBasicsTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(tuple3dBasicsTest);
      }
   }
}
