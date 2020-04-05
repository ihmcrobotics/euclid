package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class BoundingBox2DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345345L);

      Point2D min = new Point2D();
      Point2D max = new Point2D();
      BoundingBox2D boundingBox = new BoundingBox2D();
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(boundingBox.getMinPoint());
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(boundingBox.getMaxPoint());

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox2D(min, max);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox2D(new BoundingBox2D(min, max));
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      double[] minArray = new double[3];
      double[] maxArray = new double[3];
      min.get(minArray);
      max.get(maxArray);
      boundingBox = new BoundingBox2D(minArray, maxArray);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox2D(min.getX(), min.getY(), max.getX(), max.getY());
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Asserts exception is thrown when attempting to create a bad bounding box
      try
      {
         new BoundingBox2D(new double[] {1.0, 0.0}, new double[] {0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox2D(new double[] {0.0, 1.0}, new double[] {0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         new BoundingBox2D(new Point2D(1.0, 0.0), new Point2D(0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox2D(new Point2D(0.0, 1.0), new Point2D(0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         new BoundingBox2D(1.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox2D(0.0, 1.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      new BoundingBox2D(new double[] {1.0, 1.0}, new double[] {1.0, 1.0});
      new BoundingBox2D(new Point2D(1.0, 1.0), new Point2D(1.0, 1.0));
      new BoundingBox2D(1.0, 1.0, 1.0, 1.0);
   }

   @Test
   public void testStaticConstructors() throws Exception
   {
      Random random = new Random(32443L);

      Point2D min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      Point2D max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      Point2D center = new Point2D();
      center.add(min, max);
      center.scale(0.5);
      Vector2D halfSize = new Vector2D();
      halfSize.sub(max, min);
      halfSize.scale(0.5);
      BoundingBox2D boundingBox = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox2D boundingBoxOne = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D boundingBoxTwo = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D expected = new BoundingBox2D();
         expected.combine(boundingBoxOne, boundingBoxTwo);
         BoundingBox2D actual = BoundingBox2D.union(boundingBoxOne, boundingBoxTwo);

         EuclidGeometryTestTools.assertBoundingBox2DEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetMin() throws Exception
   {
      Random random = new Random(3242L);
      BoundingBox2D boundingBox = new BoundingBox2D();
      Point2D min = new Point2D();

      min = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 0.0);
      boundingBox.setMin(min);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 0.0);
      boundingBox.setMin(new double[] {min.getX(), min.getY()});
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, -10.0, 0.0);
      boundingBox.setMin(min.getX(), min.getY());
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);

      // Check exceptions
      boundingBox.setToZero();
      try
      {
         boundingBox.setMin(new double[] {1.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new double[] {0.0, 1.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMin(new Point2D(1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new Point2D(0.0, 1.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMin(1.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(0.0, 1.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.setMin(new double[] {0.0, 0.0});
      boundingBox.setMin(new Point2D(0.0, 0.0));
      boundingBox.setMin(0.0, 0.0);
   }

   @Test
   public void testSetMax() throws Exception
   {
      Random random = new Random(3242L);
      BoundingBox2D boundingBox = new BoundingBox2D();
      Point2D max = new Point2D();

      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      boundingBox.setMax(max);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      boundingBox.setMax(new double[] {max.getX(), max.getY()});
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      boundingBox.setMax(max.getX(), max.getY());
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Check exceptions
      boundingBox.setToZero();
      try
      {
         boundingBox.setMax(new double[] {-1.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new double[] {0.0, -1.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMax(new Point2D(-1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new Point2D(0.0, -1.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMax(-1.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(0.0, -1.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.setMax(new double[] {0.0, 0.0});
      boundingBox.setMax(new Point2D(0.0, 0.0));
      boundingBox.setMax(0.0, 0.0);
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(34534L);
      BoundingBox2D boundingBox = new BoundingBox2D();
      Point2D min = new Point2D();
      Point2D max = new Point2D();

      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(min.getX(), min.getY(), max.getX(), max.getY());
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      double[] minArray = new double[3];
      double[] maxArray = new double[3];
      min.get(minArray);
      max.get(maxArray);
      boundingBox.set(minArray, maxArray);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(min, max);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      Point2D center = new Point2D();
      center.add(min, max);
      center.scale(0.5);
      Vector2D halfSize = new Vector2D();
      halfSize.sub(max, min);
      halfSize.scale(0.5);
      boundingBox.set(center, halfSize);
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      max = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(new BoundingBox2D(min, max));
      EuclidCoreTestTools.assertTuple2DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple2DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Asserts exception is thrown when attempting to create a bad bounding box
      try
      {
         boundingBox.set(new double[] {1.0, 0.0}, new double[] {0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new double[] {0.0, 1.0}, new double[] {0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.set(new Point2D(1.0, 0.0), new Point2D(0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point2D(0.0, 1.0), new Point2D(0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.set(new Point2D(1.0, 0.0), new Vector2D(-1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point2D(1.0, 0.0), new Vector2D(0.0, -1.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.set(1.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(0.0, 1.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.set(new double[] {1.0, 1.0}, new double[] {1.0, 1.0});
      boundingBox.set(new Point2D(1.0, 1.0), new Point2D(1.0, 1.0));
      boundingBox.set(1.0, 1.0, 1.0, 1.0);
   }

   @Test
   public void testCombine() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox2D boundingBoxOne = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D boundingBoxTwo = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D combined = new BoundingBox2D();
         combined.combine(boundingBoxOne, boundingBoxTwo);

         // First assert that the resulting bounding box contains all the point of the two bounding bounding boxes
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMaxPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMaxPoint()));

         // Check that each coordinate of the resulting bounding box comes from one of the two original bounding boxes
         Point2DReadOnly[] originalMinCoordinates = {boundingBoxOne.getMinPoint(), boundingBoxTwo.getMinPoint()};
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            boolean isMinCoordinateFromOriginal = false;
            for (Point2DReadOnly originalCoordinate : originalMinCoordinates)
            {
               if (combined.getMinPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMinCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue(isMinCoordinateFromOriginal, "Unexpected min coordinate for the combined bounding box, axis index = " + axisIndex);
         }

         Point2DReadOnly[] originalMaxCoordinates = {boundingBoxOne.getMaxPoint(), boundingBoxTwo.getMaxPoint()};
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            boolean isMaxCoordinateFromOriginal = false;
            for (Point2DReadOnly originalCoordinate : originalMaxCoordinates)
            {
               if (combined.getMaxPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMaxCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue(isMaxCoordinateFromOriginal, "Unexpected max coordinate for the combined bounding box, axis index = " + axisIndex);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox2D boundingBoxOne = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D boundingBoxTwo = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D combined = new BoundingBox2D();
         combined.set(boundingBoxOne);
         combined.combine(boundingBoxTwo);

         // First assert that the resulting bounding box contains all the point of the two bounding bounding boxes
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMaxPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMaxPoint()));

         // Check that each coordinate of the resulting bounding box comes from one of the two original bounding boxes
         Point2DReadOnly[] originalMinCoordinates = {boundingBoxOne.getMinPoint(), boundingBoxTwo.getMinPoint()};
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            boolean isMinCoordinateFromOriginal = false;
            for (Point2DReadOnly originalCoordinate : originalMinCoordinates)
            {
               if (combined.getMinPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMinCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue(isMinCoordinateFromOriginal, "Unexpected min coordinate for the combined bounding box, axis index = " + axisIndex);
         }

         Point2DReadOnly[] originalMaxCoordinates = {boundingBoxOne.getMaxPoint(), boundingBoxTwo.getMaxPoint()};
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            boolean isMaxCoordinateFromOriginal = false;
            for (Point2DReadOnly originalCoordinate : originalMaxCoordinates)
            {
               if (combined.getMaxPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMaxCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue(isMaxCoordinateFromOriginal, "Unexpected max coordinate for the combined bounding box, axis index = " + axisIndex);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that combine is equivalent to set when one of the 2 BBX is NaN
         BoundingBox2D boundingBoxOne = new BoundingBox2D();
         boundingBoxOne.setToNaN();
         BoundingBox2D boundingBoxTwo = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D combined = new BoundingBox2D();
         combined.set(boundingBoxOne);
         combined.combine(boundingBoxTwo);

         EuclidGeometryTestTools.assertBoundingBox2DEquals(boundingBoxTwo, combined, EPSILON);

         boundingBoxOne = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         boundingBoxTwo.setToNaN();
         combined = new BoundingBox2D();
         combined.set(boundingBoxOne);
         combined.combine(boundingBoxTwo);

         EuclidGeometryTestTools.assertBoundingBox2DEquals(boundingBoxOne, combined, EPSILON);
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(453453L);
      BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
      boundingBox2D.setToNaN();
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(boundingBox2D.getMinPoint());
      EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(boundingBox2D.getMaxPoint());
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(453453L);
      BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
      boundingBox2D.setToZero();
      EuclidCoreTestTools.assertTuple2DIsSetToZero(boundingBox2D.getMinPoint());
      EuclidCoreTestTools.assertTuple2DIsSetToZero(boundingBox2D.getMaxPoint());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Random random = new Random(23434L);
      for (int i = 0; i < ITERATIONS; i++)
         assertFalse(EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0).containsNaN());

      assertTrue(new BoundingBox2D(Double.NaN, 0, 0, 0).containsNaN());
      assertTrue(new BoundingBox2D(0, Double.NaN, 0, 0).containsNaN());
      assertTrue(new BoundingBox2D(0, 0, Double.NaN, 0).containsNaN());
      assertTrue(new BoundingBox2D(0, 0, 0, Double.NaN).containsNaN());
   }

   @Test
   public void testGetCenterPoint() throws Exception
   {
      Random random = new Random(24324L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D boundingBox2D = new BoundingBox2D();
         boundingBox2D.set(center, halfSize);
         Point2D actualCenter = new Point2D();
         boundingBox2D.getCenterPoint(actualCenter);
         EuclidCoreTestTools.assertTuple2DEquals(center, actualCenter, EPSILON);
         Vector2D centerToMin = new Vector2D();
         centerToMin.sub(boundingBox2D.getMinPoint(), center);
         Vector2D centerToMax = new Vector2D();
         centerToMax.sub(boundingBox2D.getMaxPoint(), center);

         centerToMax.negate();
         EuclidCoreTestTools.assertTuple2DEquals(centerToMax, centerToMin, EPSILON);
      }
   }

   @Test
   public void testGetPointGivenParameters() throws Exception
   {
      Random random = new Random(324432L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D expectedPoint = new Point2D();
         Point2D actualPoint = new Point2D();
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         double alpha = EuclidCoreRandomTools.nextDouble(random, 10.0);

         expectedPoint.interpolate(boundingBox2D.getMinPoint(), boundingBox2D.getMaxPoint(), alpha);
         boundingBox2D.getPointGivenParameters(alpha, alpha, actualPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedPoint, actualPoint, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D actualPoint = new Point2D();
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         Point2D alpha = new Point2D();

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            alpha.setElement(axisIndex, 0.0);
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), actualPoint);
            assertEquals(boundingBox2D.getMinPoint().getElement(axisIndex), actualPoint.getElement(axisIndex), EPSILON);

            alpha = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            alpha.setElement(axisIndex, 1.0);
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), actualPoint);
            assertEquals(boundingBox2D.getMaxPoint().getElement(axisIndex), actualPoint.getElement(axisIndex), EPSILON);
         }
      }
   }

   @Test
   public void testGetDiagonalLengthSquared() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D boundingBox2D = new BoundingBox2D();
         boundingBox2D.set(center, halfSize);
         assertEquals(4.0 * halfSize.lengthSquared(), boundingBox2D.getDiagonalLengthSquared(), EPSILON);
      }
   }

   @Test
   public void testIsInsideExclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         Point2DReadOnly minPoint = boundingBox2D.getMinPoint();
         Point2DReadOnly maxPoint = boundingBox2D.getMaxPoint();
         assertFalse(boundingBox2D.isInsideExclusive(minPoint));
         assertFalse(boundingBox2D.isInsideExclusive(maxPoint));
         assertFalse(boundingBox2D.isInsideExclusive(minPoint.getX(), minPoint.getY()));
         assertFalse(boundingBox2D.isInsideExclusive(maxPoint.getX(), maxPoint.getY()));

         Point2D alpha = EuclidCoreRandomTools.nextPoint2D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
         Point2D query = new Point2D();
         boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
         assertTrue(boundingBox2D.isInsideExclusive(query));
         assertTrue(boundingBox2D.isInsideExclusive(query.getX(), query.getY()));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.nextPoint2D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertFalse(boundingBox2D.isInsideExclusive(query));
            assertFalse(boundingBox2D.isInsideExclusive(query.getX(), query.getY()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertFalse(boundingBox2D.isInsideExclusive(query));
            assertFalse(boundingBox2D.isInsideExclusive(query.getX(), query.getY()));

            alpha = EuclidCoreRandomTools.nextPoint2D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertFalse(boundingBox2D.isInsideExclusive(query));
            assertFalse(boundingBox2D.isInsideExclusive(query.getX(), query.getY()));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertFalse(boundingBox2D.isInsideExclusive(query));
            assertFalse(boundingBox2D.isInsideExclusive(query.getX(), query.getY()));
         }
      }
   }

   @Test
   public void testIsInsideInclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         Point2DReadOnly minPoint = boundingBox2D.getMinPoint();
         Point2DReadOnly maxPoint = boundingBox2D.getMaxPoint();
         assertTrue(boundingBox2D.isInsideInclusive(minPoint));
         assertTrue(boundingBox2D.isInsideInclusive(maxPoint));
         assertTrue(boundingBox2D.isInsideInclusive(minPoint.getX(), minPoint.getY()));
         assertTrue(boundingBox2D.isInsideInclusive(maxPoint.getX(), maxPoint.getY()));

         Point2D alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
         Point2D query = new Point2D();
         boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
         assertTrue(boundingBox2D.isInsideInclusive(query));
         assertTrue(boundingBox2D.isInsideInclusive(query.getX(), query.getY()));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertFalse(boundingBox2D.isInsideInclusive(query));
            assertFalse(boundingBox2D.isInsideInclusive(query.getX(), query.getY()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertFalse(boundingBox2D.isInsideInclusive(query));
            assertFalse(boundingBox2D.isInsideInclusive(query.getX(), query.getY()));

            alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
            boundingBox2D.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertTrue(boundingBox2D.isInsideInclusive(query));
            assertTrue(boundingBox2D.isInsideInclusive(query.getX(), query.getY()));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertTrue(boundingBox2D.isInsideInclusive(query));
            assertTrue(boundingBox2D.isInsideInclusive(query.getX(), query.getY()));
         }
      }
   }

   @Test
   public void testIsInsideEpsilon() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D boundingBoxEpsilon = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);
         double epsilon = EuclidCoreRandomTools.nextDouble(random, -Math.min(halfSize.getX(), halfSize.getY()), 1.0);
         halfSize.add(epsilon, epsilon);
         BoundingBox2D boundingBoxExclusive = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point2DReadOnly minPoint = boundingBoxEpsilon.getMinPoint();
         Point2DReadOnly maxPoint = boundingBoxEpsilon.getMaxPoint();
         assertEquals(boundingBoxExclusive.isInsideExclusive(minPoint), boundingBoxEpsilon.isInsideEpsilon(minPoint, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(maxPoint), boundingBoxEpsilon.isInsideEpsilon(maxPoint, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(minPoint), boundingBoxEpsilon.isInsideEpsilon(minPoint.getX(), minPoint.getY(), epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(maxPoint), boundingBoxEpsilon.isInsideEpsilon(maxPoint.getX(), maxPoint.getY(), epsilon));

         Point2D alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
         Point2D query = new Point2D();
         boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
         assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), epsilon));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), epsilon));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), epsilon));

            alpha = EuclidCoreRandomTools.nextPoint2D(random, 0.0, 1.0);
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), epsilon));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), epsilon));
         }

         // Simple tests to verify the exclusive feature
         assertFalse(boundingBoxEpsilon.isInsideEpsilon(boundingBoxEpsilon.getMaxPoint(), 0.0));
         assertFalse(boundingBoxEpsilon.isInsideEpsilon(boundingBoxEpsilon.getMinPoint(), 0.0));
      }
   }

   @Test
   public void testIntersectsExclusive() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D boundingBox2D = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point2D queryCenter = new Point2D();
         Vector2D queryHalfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D query = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         boundingBox2D.getPointGivenParameters(xParameter, yParameter, queryCenter);
         query.set(queryCenter, queryHalfSize);
         assertTrue(boundingBox2D.intersectsExclusive(query));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift = new Vector2D();
               shift.add(queryHalfSize, halfSize);
               shift.scale(xSign, ySign);

               queryCenter.scaleAdd(0.999, shift, center);
               query.set(queryCenter, queryHalfSize);
               assertTrue(boundingBox2D.intersectsExclusive(query));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter.scaleAdd(1.0001, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertFalse(boundingBox2D.intersectsExclusive(query));
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 1.0);
         assertFalse(boundingBox2D.intersectsExclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, -1.0, 1.0, 0.0);
         assertFalse(boundingBox2D.intersectsExclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(1.0, 0.0, 2.0, 1.0);
         assertFalse(boundingBox2D.intersectsExclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox2D.intersectsExclusive(query));
      }
   }

   @Test
   public void testIntersectsInclusive() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D boundingBox2D = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point2D queryCenter = new Point2D();
         Vector2D queryHalfSize = EuclidCoreRandomTools.nextVector2D(random, 0.0, 10.0);
         BoundingBox2D query = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         boundingBox2D.getPointGivenParameters(xParameter, yParameter, queryCenter);
         query.set(queryCenter, queryHalfSize);
         assertTrue(boundingBox2D.intersectsInclusive(query));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift = new Vector2D();
               shift.add(queryHalfSize, halfSize);
               shift.scale(xSign, ySign);

               queryCenter.scaleAdd(0.999, shift, center);
               query.set(queryCenter, queryHalfSize);
               assertTrue(boundingBox2D.intersectsInclusive(query));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter.scaleAdd(1.0001, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertFalse(boundingBox2D.intersectsInclusive(query));
               }
            }
         }

         // Simple tests to verify the inclusive feature
         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 1.0);
         assertTrue(boundingBox2D.intersectsInclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, -1.0, 1.0, 0.0);
         assertTrue(boundingBox2D.intersectsInclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(1.0, 0.0, 2.0, 1.0);
         assertTrue(boundingBox2D.intersectsInclusive(query));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, 1.0, 1.0, 2.0);
         assertTrue(boundingBox2D.intersectsInclusive(query));
      }
   }

   @Test
   public void testIntersectsEpsilon() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0);

         Point2D center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D halfSize = EuclidCoreRandomTools.nextVector2D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox2D boundingBox2D = BoundingBox2D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point2D queryCenter = new Point2D();
         Vector2D queryHalfSize = EuclidCoreRandomTools.nextVector2D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox2D query = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         boundingBox2D.getPointGivenParameters(xParameter, yParameter, queryCenter);
         query.set(queryCenter, queryHalfSize);
         assertTrue(boundingBox2D.intersectsEpsilon(query, epsilon));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift = new Vector2D();
               shift.add(queryHalfSize, halfSize);
               shift.add(epsilon, epsilon);
               shift.scale(xSign, ySign);

               queryCenter.scaleAdd(0.999, shift, center);
               query.set(queryCenter, queryHalfSize);
               assertTrue(boundingBox2D.intersectsEpsilon(query, epsilon));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter.scaleAdd(1.0001, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertFalse(boundingBox2D.intersectsEpsilon(query, epsilon));
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 1.0);
         assertFalse(boundingBox2D.intersectsEpsilon(query, 0.0));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, -1.0, 1.0, 0.0);
         assertFalse(boundingBox2D.intersectsEpsilon(query, 0.0));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(1.0, 0.0, 2.0, 1.0);
         assertFalse(boundingBox2D.intersectsEpsilon(query, 0.0));

         boundingBox2D.set(0.0, 0.0, 1.0, 1.0);
         query.set(0.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox2D.intersectsEpsilon(query, 0.0));
      }
   }

   @Test
   public void testDoesIntersectWithLine2D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly pointOnLine = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2DReadOnly lineDirection = EuclidCoreRandomTools.nextVector2D(random);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                          boundingBox2D.getMaxPoint(),
                                                                                          pointOnLine,
                                                                                          lineDirection,
                                                                                          null,
                                                                                          null) != 0;
         boolean actual = boundingBox2D.doesIntersectWithLine2D(pointOnLine, lineDirection);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testDoesIntersectWithLineSegment2D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2DReadOnly lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                                 boundingBox2D.getMaxPoint(),
                                                                                                 lineSegmentStart,
                                                                                                 lineSegmentEnd,
                                                                                                 null,
                                                                                                 null) != 0;
         boolean actual = boundingBox2D.doesIntersectWithLineSegment2D(lineSegmentStart, lineSegmentEnd);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testDoesIntersectWithRay2D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2DReadOnly rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                         boundingBox2D.getMaxPoint(),
                                                                                         rayOrigin,
                                                                                         rayDirection,
                                                                                         null,
                                                                                         null) != 0;
         boolean actual = boundingBox2D.doesIntersectWithRay2D(rayOrigin, rayDirection);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testIntersectionWithLine2D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly pointOnLine = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2DReadOnly lineDirection = EuclidCoreRandomTools.nextVector2D(random);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point2D expectedFirstIntersection = new Point2D();
         Point2D expectedSecondIntersection = new Point2D();
         Point2D actualFirstIntersection = new Point2D();
         Point2D actualSecondIntersection = new Point2D();
         expectedN = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                   boundingBox2D.getMaxPoint(),
                                                                                   pointOnLine,
                                                                                   lineDirection,
                                                                                   expectedFirstIntersection,
                                                                                   expectedSecondIntersection);
         actualN = boundingBox2D.intersectionWithLine2D(pointOnLine, lineDirection, actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testintersectionWithLineSegment2D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2DReadOnly lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point2D expectedFirstIntersection = new Point2D();
         Point2D expectedSecondIntersection = new Point2D();
         Point2D actualFirstIntersection = new Point2D();
         Point2D actualSecondIntersection = new Point2D();
         expectedN = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                          boundingBox2D.getMaxPoint(),
                                                                                          lineSegmentStart,
                                                                                          lineSegmentEnd,
                                                                                          expectedFirstIntersection,
                                                                                          expectedSecondIntersection);
         actualN = boundingBox2D.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testintersectionWithRay2D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2DReadOnly rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2DReadOnly rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point2D expectedFirstIntersection = new Point2D();
         Point2D expectedSecondIntersection = new Point2D();
         Point2D actualFirstIntersection = new Point2D();
         Point2D actualSecondIntersection = new Point2D();
         expectedN = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBox2D.getMinPoint(),
                                                                                  boundingBox2D.getMaxPoint(),
                                                                                  rayOrigin,
                                                                                  rayDirection,
                                                                                  expectedFirstIntersection,
                                                                                  expectedSecondIntersection);
         actualN = boundingBox2D.intersectionWithRay2D(rayOrigin, rayDirection, actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);

         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple2DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testUpdateToIncludePoint() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test updateToIncludePoint(Point2DReadOnly point)
         BoundingBox2D original = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D extended = new BoundingBox2D(original);

         Point2D point = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         extended.updateToIncludePoint(point);

         if (original.isInsideInclusive(point))
         { // Make sure calling it did not change the bounding box
            assertEquals(original, extended);
         }
         else
         { // If a bound has changed, it must be set to the point coordinate.
            assertTrue(extended.isInsideInclusive(point));

            Point2DReadOnly originalMin = original.getMinPoint();
            Point2DReadOnly extendedMin = extended.getMinPoint();

            for (int axis = 0; axis < 2; axis++)
            {
               boolean hasNotChanged = originalMin.getElement(axis) == extendedMin.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMin.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }

            Point2DReadOnly originalMax = original.getMaxPoint();
            Point2DReadOnly extendedMax = extended.getMaxPoint();

            for (int axis = 0; axis < 2; axis++)
            {
               boolean hasNotChanged = originalMax.getElement(axis) == extendedMax.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMax.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test updateToIncludePoint(Point2DReadOnly point)
         BoundingBox2D original = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
         BoundingBox2D extended = new BoundingBox2D(original);

         Point2D point = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         extended.updateToIncludePoint(point.getX(), point.getY());

         if (original.isInsideInclusive(point))
         { // Make sure calling it did not change the bounding box
            assertEquals(original, extended);
         }
         else
         { // If a bound has changed, it must be set to the point coordinate.
            assertTrue(extended.isInsideInclusive(point));

            Point2DReadOnly originalMin = original.getMinPoint();
            Point2DReadOnly extendedMin = extended.getMinPoint();

            for (int axis = 0; axis < 2; axis++)
            {
               boolean hasNotChanged = originalMin.getElement(axis) == extendedMin.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMin.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }

            Point2DReadOnly originalMax = original.getMaxPoint();
            Point2DReadOnly extendedMax = extended.getMaxPoint();

            for (int axis = 0; axis < 2; axis++)
            {
               boolean hasNotChanged = originalMax.getElement(axis) == extendedMax.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMax.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }
         }
      }
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(3242L);

      BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
      Point2DReadOnly min = boundingBox2D.getMinPoint();
      Point2DReadOnly max = boundingBox2D.getMaxPoint();

      {
         assertTrue(min.getX() == boundingBox2D.getMinX());
         assertTrue(min.getY() == boundingBox2D.getMinY());
         assertTrue(max.getX() == boundingBox2D.getMaxX());
         assertTrue(max.getY() == boundingBox2D.getMaxY());
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(234234L);
      double epsilon = random.nextDouble();
      BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
      double minX = boundingBox2D.getMinX();
      double minY = boundingBox2D.getMinY();
      double maxX = boundingBox2D.getMaxX();
      double maxY = boundingBox2D.getMaxY();

      double small = 0.999 * epsilon;
      double big = 1.001 * epsilon;

      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX + small, minY, maxX, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY + small, maxX, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX + small, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX, maxY + small), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX - small, minY, maxX, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY - small, maxX, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX - small, maxY), epsilon));
      assertTrue(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX, maxY - small), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX + big, minY, maxX, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY + big, maxX, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX + big, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX, maxY + big), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX - big, minY, maxX, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY - big, maxX, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX - big, maxY), epsilon));
      assertFalse(boundingBox2D.epsilonEquals(new BoundingBox2D(minX, minY, maxX, maxY - big), epsilon));
   }

   @SuppressWarnings("unlikely-arg-type")
   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(234234L);
      BoundingBox2D boundingBox2D = EuclidGeometryRandomTools.nextBoundingBox2D(random, 10.0, 10.0);
      double minX = boundingBox2D.getMinX();
      double minY = boundingBox2D.getMinY();
      double maxX = boundingBox2D.getMaxX();
      double maxY = boundingBox2D.getMaxY();

      double smallestEpsilon = 8.8888e-16;

      assertTrue(boundingBox2D.equals(new BoundingBox2D(minX, minY, maxX, maxY)));
      Object bbxAxObject = new BoundingBox2D(minX, minY, maxX, maxY);
      assertTrue(boundingBox2D.equals(bbxAxObject));
      assertFalse(boundingBox2D.equals(null));
      assertFalse(boundingBox2D.equals(new double[5]));

      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX + smallestEpsilon, minY, maxX, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY + smallestEpsilon, maxX, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY, maxX + smallestEpsilon, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY, maxX, maxY + smallestEpsilon)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX - smallestEpsilon, minY, maxX, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY - smallestEpsilon, maxX, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY, maxX - smallestEpsilon, maxY)));
      assertFalse(boundingBox2D.equals(new BoundingBox2D(minX, minY, maxX, maxY - smallestEpsilon)));
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(987234L);
      BoundingBox2D firstBox, secondBox;

      {
         Point2D firstPoint = EuclidCoreRandomTools.nextPoint2D(random, 0.1, 2.5);
         Point2D secondPoint = EuclidCoreRandomTools.nextPoint2D(random, 2.5, 5.0);

         firstBox = new BoundingBox2D(firstPoint, secondPoint);
         secondBox = new BoundingBox2D(firstBox);

         assertTrue(firstBox.geometricallyEquals(secondBox, EPSILON));
         assertTrue(secondBox.geometricallyEquals(firstBox, EPSILON));
         assertTrue(firstBox.geometricallyEquals(firstBox, EPSILON));
         assertTrue(secondBox.geometricallyEquals(secondBox, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; ++i)
      {
         Point2D min = EuclidCoreRandomTools.nextPoint2D(random, 0.1, 2.5);
         Point2D max = EuclidCoreRandomTools.nextPoint2D(random, 2.5, 5.0);

         firstBox = new BoundingBox2D(min, max);

         Point2D minCorrupted = new Point2D(min);
         minCorrupted.add(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON));
         secondBox = new BoundingBox2D(minCorrupted, max);
         assertTrue(firstBox.geometricallyEquals(secondBox, EPSILON));

         minCorrupted = new Point2D(min);
         minCorrupted.add(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON));
         secondBox = new BoundingBox2D(minCorrupted, max);
         assertFalse(firstBox.geometricallyEquals(secondBox, EPSILON));

         Point2D maxCorrupted = new Point2D(max);
         maxCorrupted.add(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * EPSILON));
         secondBox = new BoundingBox2D(min, maxCorrupted);
         assertTrue(firstBox.geometricallyEquals(secondBox, EPSILON));

         maxCorrupted = new Point2D(max);
         maxCorrupted.add(EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * EPSILON));
         secondBox = new BoundingBox2D(min, maxCorrupted);
         assertFalse(firstBox.geometricallyEquals(secondBox, EPSILON));
      }
   }
}
