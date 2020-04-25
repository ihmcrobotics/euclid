package us.ihmc.euclid.referenceFrame.tools;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox2D;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameRotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameUnitVector2D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.api.RandomFrameTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreFactoriesTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class EuclidFrameFactoriesTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testNewLinkedFramePoint2DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedFramePoint2DReadOnly(DoubleSupplier scaleSupplier, FrameTuple2DReadOnly originalTuple)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         double[] scale = new double[1];
         FramePoint2D originalTuple = new FramePoint2D();
         FramePoint2DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint2DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            scale[0] = random.nextDouble();

            FramePoint2D expected = new FramePoint2D(originalTuple);
            expected.scale(scale[0]);
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }

      { // Test newLinkedFramePoint2DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier, DoubleSupplier ySupplier)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint2D expected = new FramePoint2D();
         FramePoint2DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint2DReadOnly(expected::getReferenceFrame, expected::getX, expected::getY);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }

      { // Test newLinkedFramePoint2DReadOnly(ReferenceFrameHolder referenceFrameHolder, Point2DReadOnly point)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint2D expected = new FramePoint2D();
         FramePoint2DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint2DReadOnly(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedFrameVector2DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedFrameVector2DReadOnly(DoubleSupplier scaleSupplier, FrameTuple2DReadOnly originalTuple)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         double[] scale = new double[1];
         FrameVector2D originalTuple = new FrameVector2D();
         FrameVector2DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector2DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            scale[0] = random.nextDouble();

            FrameVector2D expected = new FrameVector2D(originalTuple);
            expected.scale(scale[0]);
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }

      { // Test newLinkedFrameVector2DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier, DoubleSupplier ySupplier)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector2D expected = new FrameVector2D();
         FrameVector2DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector2DReadOnly(expected::getReferenceFrame, expected::getX, expected::getY);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }

      { // Test newLinkedFrameVector2DReadOnly(ReferenceFrameHolder referenceFrameHolder, Vector2DReadOnly point)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector2D expected = new FrameVector2D();
         FrameVector2DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector2DReadOnly(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedFramePoint3DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedFramePoint3DReadOnly(DoubleSupplier scaleSupplier, FrameTuple3DReadOnly originalTuple)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         double[] scale = new double[1];
         FramePoint3D originalTuple = new FramePoint3D();
         FramePoint3DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            scale[0] = random.nextDouble();

            FramePoint3D expected = new FramePoint3D(originalTuple);
            expected.scale(scale[0]);
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }

      { // Test newLinkedFramePoint3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint3D expected = new FramePoint3D();
         FramePoint3DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(expected::getReferenceFrame,
                                                                                          expected::getX,
                                                                                          expected::getY,
                                                                                          expected::getZ);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }

      { // Test newLinkedFramePoint3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Point3DReadOnly point)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint3D expected = new FramePoint3D();
         FramePoint3DReadOnly actual = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedFrameVector3DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedFrameVector3DReadOnly(DoubleSupplier scaleSupplier, FrameTuple3DReadOnly originalTuple)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         double[] scale = new double[1];
         FrameVector3D originalTuple = new FrameVector3D();
         FrameVector3DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> scale[0], originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            scale[0] = random.nextDouble();

            FrameVector3D expected = new FrameVector3D(originalTuple);
            expected.scale(scale[0]);
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }

      { // Test newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector3D expected = new FrameVector3D();
         FrameVector3DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(expected::getReferenceFrame,
                                                                                            expected::getX,
                                                                                            expected::getY,
                                                                                            expected::getZ);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }

      { // Test newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly point)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector3D expected = new FrameVector3D();
         FrameVector3DReadOnly actual = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedFramePoint2D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedFramePoint2D(FramePoint2DReadOnly originalPoint)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint2D originalTuple = new FramePoint2D();
         FramePoint2DReadOnly actual = EuclidFrameFactories.newNegativeLinkedFramePoint2D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            FramePoint2D expected = new FramePoint2D(originalTuple);
            expected.negate();

            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedFrameVector2D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedFrameVector2D(FrameVector2DReadOnly originalVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector2D originalTuple = new FrameVector2D();
         FrameVector2DReadOnly actual = EuclidFrameFactories.newNegativeLinkedFrameVector2D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            FrameVector2D expected = new FrameVector2D(originalTuple);
            expected.negate();

            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedFramePoint3D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedFramePoint3D(FramePoint3DReadOnly originalPoint)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint3D originalTuple = new FramePoint3D();
         FramePoint3DReadOnly actual = EuclidFrameFactories.newNegativeLinkedFramePoint3D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            FramePoint3D expected = new FramePoint3D(originalTuple);
            expected.negate();

            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewNegativeLinkedFrameVector3D()
   {
      Random random = new Random(43);

      { // Test newNegativeLinkedFrameVector3D(FrameVector3DReadOnly originalVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector3D originalTuple = new FrameVector3D();
         FrameVector3DReadOnly actual = EuclidFrameFactories.newNegativeLinkedFrameVector3D(originalTuple);

         for (int i = 0; i < ITERATIONS; i++)
         {
            originalTuple.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            FrameVector3D expected = new FrameVector3D(originalTuple);
            expected.negate();

            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFramePoint2DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFramePoint2DBasics copy = EuclidFrameFactories.newFixedFramePoint2DBasics(() -> referenceFrame);
         copy.set((Tuple2DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFramePoint2DBasics next = EuclidFrameFactories.newFixedFramePoint2DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextPoint2D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextPoint2D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFramePoint2DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFramePoint2DBasics(ReferenceFrameHolder referenceFrameHolder, Point2DBasics originalPoint)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint2D expected = new FramePoint2D();
         FixedFramePoint2DBasics actual = EuclidFrameFactories.newLinkedFixedFramePoint2DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextPoint2D(random));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameVector2DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameVector2DBasics copy = EuclidFrameFactories.newFixedFrameVector2DBasics(() -> referenceFrame);
         copy.set((Tuple2DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameVector2DBasics next = EuclidFrameFactories.newFixedFrameVector2DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextVector2D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextVector2D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameVector2DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFrameVector2DBasics(ReferenceFrameHolder referenceFrameHolder, Vector2DBasics originalVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector2D expected = new FrameVector2D();
         FixedFrameVector2DBasics actual = EuclidFrameFactories.newLinkedFixedFrameVector2DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextVector2D(random));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedFixedFramePoint3DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFramePoint3DBasics(ReferenceFrameHolder referenceFrameHolder, Point3DBasics originalPoint)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FramePoint3D expected = new FramePoint3D();
         FixedFramePoint3DBasics actual = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextPoint3D(random));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameVector3DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameVector3DBasics copy = EuclidFrameFactories.newFixedFrameVector3DBasics(() -> referenceFrame);
         copy.set((Tuple3DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameVector3DBasics next = EuclidFrameFactories.newFixedFrameVector3DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextVector3D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextVector3D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameVector3DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder, Vector3DBasics originalVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameVector3D expected = new FrameVector3D();
         FixedFrameVector3DBasics actual = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextVector3D(random));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameUnitVector2DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameUnitVector2DBasics copy = EuclidFrameFactories.newFixedFrameUnitVector2DBasics(() -> referenceFrame);
         copy.set((Tuple2DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameUnitVector2DBasics next = EuclidFrameFactories.newFixedFrameUnitVector2DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextUnitVector2D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextUnitVector2D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameUnitVector2DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFrameUnitVector2DBasics(ReferenceFrameHolder referenceFrameHolder, UnitVector2DBasics originalUnitVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameUnitVector2D expected = new FrameUnitVector2D();
         FixedFrameUnitVector2DBasics actual = EuclidFrameFactories.newLinkedFixedFrameUnitVector2DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameUnitVector2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextUnitVector2D(random));
            thoroughAssertionsFrameTuple2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameUnitVector3DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameUnitVector3DBasics copy = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(() -> referenceFrame);
         copy.set((Tuple3DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameUnitVector3DBasics next = EuclidFrameFactories.newFixedFrameUnitVector3DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextUnitVector3D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextUnitVector3D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameUnitVector3DBasics()
   {
      Random random = new Random(983456);

      { // Test newLinkedFixedFrameUnitVector3DBasics(ReferenceFrameHolder referenceFrameHolder, UnitVector3DBasics originalUnitVector)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameUnitVector3D expected = new FrameUnitVector3D();
         FixedFrameUnitVector3DBasics actual = EuclidFrameFactories.newLinkedFixedFrameUnitVector3DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameUnitVector3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextUnitVector3D(random));
            thoroughAssertionsFrameTuple3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameOrientation2DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameOrientation2DBasics copy = EuclidFrameFactories.newFixedFrameOrientation2DBasics(() -> referenceFrame);
         copy.set((Orientation2DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameOrientation2DBasics next = EuclidFrameFactories.newFixedFrameOrientation2DBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextOrientation2D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextOrientation2D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameOrientation2DBasics()
   {
      Random random = new Random(435346);

      { // Test newLinkedFixedFrameOrientation2DBasics(ReferenceFrameHolder referenceFrameHolder, Orientation2DBasics originalOrientation)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameOrientation2D expected = new FrameOrientation2D();
         FixedFrameOrientation2DBasics actual = EuclidFrameFactories.newLinkedFixedFrameOrientation2DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameOrientation2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameOrientation2D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextOrientation2D(random));
            thoroughAssertionsFrameOrientation2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameQuaternionBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameQuaternionBasics copy = EuclidFrameFactories.newFixedFrameQuaternionBasics(() -> referenceFrame);
         copy.set((QuaternionReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameQuaternionBasics next = EuclidFrameFactories.newFixedFrameQuaternionBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextQuaternion(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextQuaternion,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameQuaternionBasics()
   {
      Random random = new Random(435346);

      { // Test newLinkedFixedFrameQuaternionBasics(ReferenceFrameHolder referenceFrameHolder, QuaternionBasics originalQuaternion)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameQuaternion expected = new FrameQuaternion();
         FixedFrameQuaternionBasics actual = EuclidFrameFactories.newLinkedFixedFrameQuaternionBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameTuple4D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextQuaternion(random));
            thoroughAssertionsFrameTuple4D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameRotationMatrixBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameRotationMatrixBasics copy = EuclidFrameFactories.newFixedFrameRotationMatrixBasics(() -> referenceFrame);
         copy.set((RotationMatrixReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameRotationMatrixBasics next = EuclidFrameFactories.newFixedFrameRotationMatrixBasics(() -> referenceFrame);
         next.set(EuclidCoreRandomTools.nextRotationMatrix(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidCoreRandomTools::nextRotationMatrix,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameRotationMatrixBasics()
   {
      Random random = new Random(435346);

      { // Test newLinkedFixedFrameRotationMatrixBasics(ReferenceFrameHolder referenceFrameHolder, RotationMatrixBasics originalRotationMatrix)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameRotationMatrix expected = new FrameRotationMatrix();
         FixedFrameRotationMatrixBasics actual = EuclidFrameFactories.newLinkedFixedFrameRotationMatrixBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameRotationMatrix(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameMatrix3D(expected, actual);

            actual.set(EuclidCoreRandomTools.nextRotationMatrix(random));
            thoroughAssertionsFrameMatrix3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameBoudingBox2DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameBoundingBox2DBasics copy = EuclidFrameFactories.newFixedFrameBoundingBox2DBasics(() -> referenceFrame);
         copy.set((BoundingBox2DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameBoundingBox2DBasics next = EuclidFrameFactories.newFixedFrameBoundingBox2DBasics(() -> referenceFrame);
         next.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidGeometryRandomTools::nextBoundingBox2D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameBoudingBox2DBasics()
   {
      Random random = new Random(435346);

      { // Test newLinkedFixedFrameBoudingBox2DBasics(ReferenceFrameHolder referenceFrameHolder, BoudingBox2DBasics originalBoudingBox2D)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameBoundingBox2D expected = new FrameBoundingBox2D();
         FixedFrameBoundingBox2DBasics actual = EuclidFrameFactories.newLinkedFixedFrameBoundingBox2DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameBoundingBox2D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameBoundingBox2D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsFrameBoundingBox2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewFixedFrameBoudingBox3DBasics() throws Throwable
   {
      FrameTypeCopier frameTypeCopier = (referenceFrame, framelessObject) ->
      {
         FixedFrameBoundingBox3DBasics copy = EuclidFrameFactories.newFixedFrameBoundingBox3DBasics(() -> referenceFrame);
         copy.set((BoundingBox3DReadOnly) framelessObject);
         return copy;
      };
      RandomFrameTypeBuilder frameTypeBuilder = (random, referenceFrame) ->
      {
         FixedFrameBoundingBox3DBasics next = EuclidFrameFactories.newFixedFrameBoundingBox3DBasics(() -> referenceFrame);
         next.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
         return next;
      };

      EuclidFrameAPITester test = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      test.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeCopier,
                                                                EuclidGeometryRandomTools::nextBoundingBox3D,
                                                                methodFilter,
                                                                EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      methodFilter = methodFilter.and(m -> !m.getName().equals("equals"));
      methodFilter = methodFilter.and(m -> !m.getName().equals("epsilonEquals"));
      test.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frameTypeBuilder, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testNewLinkedFixedFrameBoudingBox3DBasics()
   {
      Random random = new Random(435346);

      { // Test newLinkedFixedFrameBoudingBox3DBasics(ReferenceFrameHolder referenceFrameHolder, BoudingBox3DBasics originalBoudingBox3D)
         ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         FrameBoundingBox3D expected = new FrameBoundingBox3D();
         FixedFrameBoundingBox3DBasics actual = EuclidFrameFactories.newLinkedFixedFrameBoundingBox3DBasics(expected::getReferenceFrame, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.setIncludingFrame(EuclidFrameRandomTools.nextFrameBoundingBox3D(random, EuclidCoreRandomTools.nextElementIn(random, frames)));
            thoroughAssertionsFrameBoundingBox3D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsFrameBoundingBox3D(expected, actual);
         }
      }
   }

   private void thoroughAssertionsFrameTuple2D(FrameTuple2DReadOnly expected, FrameTuple2DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameTuple2DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameTuple2DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameTuple3D(FrameTuple3DReadOnly expected, FrameTuple3DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameTuple3DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameTuple3DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameTuple4D(FrameTuple4DReadOnly expected, FrameTuple4DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameTuple4DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameTuple4DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameMatrix3D(FrameMatrix3DReadOnly expected, FrameMatrix3DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameMatrix3DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameMatrix3DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameBoundingBox2D(FrameBoundingBox2DReadOnly expected, FrameBoundingBox2DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameBoundingBox2DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameBoundingBox2DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameBoundingBox3D(FrameBoundingBox3DReadOnly expected, FrameBoundingBox3DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameBoundingBox3DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameBoundingBox3DEquals(actual, expected, EPSILON);
   }

   private void thoroughAssertionsFrameOrientation2D(FrameOrientation2DReadOnly expected, FrameOrientation2DReadOnly actual)
   {
      EuclidCoreFactoriesTest.assertObjectMethods(expected, actual);
      EuclidFrameTestTools.assertFrameOrientation2DEquals(expected, actual, EPSILON);
      EuclidFrameTestTools.assertFrameOrientation2DEquals(actual, expected, EPSILON);
   }
}
