package us.ihmc.euclid.geometry.tools;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.tools.EuclidCoreFactoriesTest.assertAllFalses;
import static us.ihmc.euclid.tools.EuclidCoreFactoriesTest.assertObjectMethods;

import java.util.EnumMap;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class EuclidGeometryFactoriesTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testNewLinkedBoundingBox2DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedBoundingBox2DReadOnly(Point2DReadOnly minPoint, Point2DReadOnly maxPoint)
         BoundingBox2D expected = new BoundingBox2D();
         BoundingBox2DReadOnly actual = EuclidGeometryFactories.newLinkedBoundingBox2DReadOnly(expected.getMinPoint(), expected.getMaxPoint());

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsBoundingBox2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedBoundingBox3DReadOnly()
   {
      Random random = new Random(5416);

      { // Test newLinkedBoundingBox3DReadOnly(Point3DReadOnly minPoint, Point3DReadOnly maxPoint)
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3DReadOnly actual = EuclidGeometryFactories.newLinkedBoundingBox3DReadOnly(expected.getMinPoint(), expected.getMaxPoint());

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsBoundingBox3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedBoundingBox2DBasics()
   {
      Random random = new Random(5416);

      { // Test newLinkedBoundingBox2DBasics(Point2DBasics minPoint, Point2DBasics maxPoint)
         BoundingBox2D expected = new BoundingBox2D();
         BoundingBox2DBasics actual = EuclidGeometryFactories.newLinkedBoundingBox2DBasics(expected.getMinPoint(), expected.getMaxPoint());

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsBoundingBox2D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsBoundingBox2D(expected, actual);
         }
      }
   }

   @Test
   public void testNewLinkedBoundingBox3DBasics()
   {
      Random random = new Random(5416);

      { // Test newLinkedBoundingBox3DBasics(Point3DBasics minPoint, Point3DBasics maxPoint)
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3DBasics actual = EuclidGeometryFactories.newLinkedBoundingBox3DBasics(expected.getMinPoint(), expected.getMaxPoint());

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsBoundingBox3D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsBoundingBox3D(expected, actual);
         }
      }
   }

   @Test
   public void testNewObservableBoundingBox2DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         BoundingBox2D expected = new BoundingBox2D();
         BoundingBox2DBasics actual = EuclidGeometryFactories.newObservableBoundingBox2DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsBoundingBox2D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox2D(random));
            thoroughAssertionsBoundingBox2D(expected, actual);
         }
      }

      { // Test with simple notification flags
         EnumMap<Bound, boolean[]> changeTraceMap = new EnumMap<>(Bound.class);
         EnumMap<Bound, boolean[]> accessTraceMap = new EnumMap<>(Bound.class);
         for (Bound bound : Bound.values)
         {
            changeTraceMap.put(bound, new boolean[] {false, false});
            accessTraceMap.put(bound, new boolean[] {false, false});
         }
         BoundingBox2DBasics source = new BoundingBox2D();
         BoundingBox2DBasics observable = EuclidGeometryFactories.newObservableBoundingBox2DBasics((axis,
                                                                                                    bound,
                                                                                                    newValue) -> changeTraceMap.get(bound)[axis.ordinal()] = true,
                                                                                                   (axis,
                                                                                                    bound) -> accessTraceMap.get(bound)[axis.ordinal()] = true,
                                                                                                   source);

         EnumMap<Bound, Point2DBasics> boundPoints = new EnumMap<>(Bound.class);
         boundPoints.put(Bound.MIN, observable.getMinPoint());
         boundPoints.put(Bound.MAX, observable.getMaxPoint());

         for (Bound bound : Bound.values)
         {
            boolean[] changeTrace = changeTraceMap.get(bound);
            boolean[] accessTraces = accessTraceMap.get(bound);
            Point2DBasics boundPoint = boundPoints.get(bound);

            assertAllFalses(changeTrace);
            assertAllFalses(accessTraces);

            for (int i = 0; i < 2; i++)
            {
               double value = random.nextDouble();
               boundPoint.setElement(i, value);
               assertTrue(changeTrace[i]);
               changeTrace[i] = false;
               boundPoint.setElement(i, value);
               assertFalse(changeTrace[i]);
               assertAllFalses(changeTrace);
               assertAllFalses(accessTraces);

               boundPoint.getElement(i);
               assertTrue(accessTraces[i]);
               accessTraces[i] = false;
               assertAllFalses(changeTrace);
               assertAllFalses(accessTraces);
            }
         }
      }
   }

   @Test
   public void testNewObservableBoundingBox3DBasics()
   {
      Random random = new Random(4367);

      { // Test the link property with the source
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3DBasics actual = EuclidGeometryFactories.newObservableBoundingBox3DBasics(null, null, expected);

         for (int i = 0; i < ITERATIONS; i++)
         {
            expected.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsBoundingBox3D(expected, actual);

            actual.set(EuclidGeometryRandomTools.nextBoundingBox3D(random));
            thoroughAssertionsBoundingBox3D(expected, actual);
         }
      }

      { // Test with simple notification flags
         EnumMap<Bound, boolean[]> changeTraceMap = new EnumMap<>(Bound.class);
         EnumMap<Bound, boolean[]> accessTraceMap = new EnumMap<>(Bound.class);
         for (Bound bound : Bound.values)
         {
            changeTraceMap.put(bound, new boolean[] {false, false, false});
            accessTraceMap.put(bound, new boolean[] {false, false, false});
         }
         BoundingBox3DBasics source = new BoundingBox3D();
         BoundingBox3DBasics observable = EuclidGeometryFactories.newObservableBoundingBox3DBasics((axis,
                                                                                                    bound,
                                                                                                    newValue) -> changeTraceMap.get(bound)[axis.ordinal()] = true,
                                                                                                   (axis,
                                                                                                    bound) -> accessTraceMap.get(bound)[axis.ordinal()] = true,
                                                                                                   source);

         EnumMap<Bound, Point3DBasics> boundPoints = new EnumMap<>(Bound.class);
         boundPoints.put(Bound.MIN, observable.getMinPoint());
         boundPoints.put(Bound.MAX, observable.getMaxPoint());

         for (Bound bound : Bound.values)
         {
            boolean[] changeTrace = changeTraceMap.get(bound);
            boolean[] accessTraces = accessTraceMap.get(bound);
            Point3DBasics boundPoint = boundPoints.get(bound);

            assertAllFalses(changeTrace);
            assertAllFalses(accessTraces);

            for (int i = 0; i < 3; i++)
            {
               double value = random.nextDouble();
               boundPoint.setElement(i, value);
               assertTrue(changeTrace[i]);
               changeTrace[i] = false;
               boundPoint.setElement(i, value);
               assertFalse(changeTrace[i]);
               assertAllFalses(changeTrace);
               assertAllFalses(accessTraces);

               boundPoint.getElement(i);
               assertTrue(accessTraces[i]);
               accessTraces[i] = false;
               assertAllFalses(changeTrace);
               assertAllFalses(accessTraces);
            }
         }
      }
   }

   public static void thoroughAssertionsBoundingBox2D(BoundingBox2DReadOnly expected, BoundingBox2DReadOnly actual)
   {
      assertObjectMethods(expected, actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      EuclidCoreTestTools.assertEquals(actual, expected, EPSILON);
   }

   public static void thoroughAssertionsBoundingBox3D(BoundingBox3DReadOnly expected, BoundingBox3DReadOnly actual)
   {
      assertObjectMethods(expected, actual);
      EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      EuclidCoreTestTools.assertEquals(actual, expected, EPSILON);
   }
}
