package us.ihmc.euclid.shape.collision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidShape3DCollisionResultTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testConstructors()
   {
      Random random = new Random(654651);

      { // Test empty constructor
         EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
         assertNull(collisionResult.getShapeA());
         assertNull(collisionResult.getShapeB());
         assertFalse(collisionResult.areShapesColliding());
         assertEquals(0.0, collisionResult.getDistance());
         assertEquals(0.0, collisionResult.getSignedDistance());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(collisionResult.getPointOnA());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(collisionResult.getNormalOnA());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(collisionResult.getPointOnB());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(collisionResult.getNormalOnB());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clone constructor
         EuclidShape3DCollisionResult expected = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult(expected);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, 0.0);
      }
   }

   @Test
   public void testSetters()
   {
      Random random = new Random(1654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(EuclidShape3DCollisionResult other)
         EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
         EuclidShape3DCollisionResult expected = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);

         collisionResult.set(expected);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, collisionResult, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(EuclidShape3DCollisionResultReadOnly other)
         EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
         EuclidShape3DCollisionResultReadOnly expected = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);

         collisionResult.set(expected);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, collisionResult, 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setShapeA(Shape3DReadOnly shapeA)
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         Shape3DReadOnly expected = EuclidShapeRandomTools.nextShape3D(random);

         tested.setShapeA(expected);

         assertEquals(expected, tested.getShapeA());
         assertEquals(original.getShapeB(), tested.getShapeB());
         assertEquals(original.areShapesColliding(), tested.areShapesColliding());
         assertEquals(original.getDistance(), tested.getDistance());
         assertEquals(original.getSignedDistance(), tested.getSignedDistance());
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnA(), tested.getPointOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnB(), tested.getPointOnB(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnA(), tested.getNormalOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnB(), tested.getNormalOnB(), 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setShapeB(Shape3DReadOnly shapeB)
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         Shape3DReadOnly expected = EuclidShapeRandomTools.nextShape3D(random);

         tested.setShapeB(expected);

         assertEquals(original.getShapeA(), tested.getShapeA());
         assertEquals(expected, tested.getShapeB());
         assertEquals(original.areShapesColliding(), tested.areShapesColliding());
         assertEquals(original.getDistance(), tested.getDistance());
         assertEquals(original.getSignedDistance(), tested.getSignedDistance());
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnA(), tested.getPointOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnB(), tested.getPointOnB(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnA(), tested.getNormalOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnB(), tested.getNormalOnB(), 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setShapesAreColliding(boolean shapesAreColliding)
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         boolean expected = random.nextBoolean();

         tested.setShapesAreColliding(expected);

         assertEquals(original.getShapeA(), tested.getShapeA());
         assertEquals(original.getShapeB(), tested.getShapeB());
         assertEquals(expected, tested.areShapesColliding());
         assertEquals(original.getDistance(), tested.getDistance());
         assertEquals(original.getSignedDistance(), tested.getSignedDistance());
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnA(), tested.getPointOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnB(), tested.getPointOnB(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnA(), tested.getNormalOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnB(), tested.getNormalOnB(), 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setSignedDistance(double distance)
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         double expected = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);

         tested.setSignedDistance(expected);

         assertEquals(Math.abs(expected), tested.getDistance());
         assertEquals(expected, tested.getSignedDistance());
         assertEquals(original.getShapeA(), tested.getShapeA());
         assertEquals(original.getShapeB(), tested.getShapeB());
         assertEquals(original.areShapesColliding(), tested.areShapesColliding());
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnA(), tested.getPointOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getPointOnB(), tested.getPointOnB(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnA(), tested.getNormalOnA(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(original.getNormalOnB(), tested.getNormalOnB(), 0.0);
      }
   }

   @Test
   public void testContainsNaN()
   {
      Random random = new Random(484);
      EuclidShape3DCollisionResult tested;

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      ((Shape3DBasics) tested.getShapeA()).setToNaN();
      assertFalse(tested.containsNaN());
      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      ((Shape3DBasics) tested.getShapeB()).setToNaN();
      assertFalse(tested.containsNaN());

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      tested.setSignedDistance(Double.NaN);
      assertTrue(tested.containsNaN());

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      tested.getPointOnA().setToNaN();
      assertTrue(tested.containsNaN());

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      tested.getPointOnB().setToNaN();
      assertTrue(tested.containsNaN());

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      tested.getNormalOnA().setToNaN();
      assertTrue(tested.containsNaN());

      tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
      assertFalse(tested.containsNaN());
      tested.getNormalOnB().setToNaN();
      assertTrue(tested.containsNaN());
   }

   @Test
   public void setToZero()
   {
      Random random = new Random(641981);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D shapeAOriginal = EuclidShapeRandomTools.nextBox3D(random);
         Box3D shapeA = new Box3D(shapeAOriginal);
         Sphere3D shapeBOriginal = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D shapeB = new Sphere3D(shapeBOriginal);
         EuclidShape3DCollisionResult tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         tested.setShapeA(shapeA);
         tested.setShapeB(shapeB);

         tested.setToZero();

         assertEquals(shapeAOriginal, shapeA);
         assertEquals(shapeBOriginal, shapeB);
         assertFalse(tested.areShapesColliding());
         assertEquals(0.0, tested.getDistance());
         assertEquals(0.0, tested.getSignedDistance());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(tested.getPointOnA());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(tested.getPointOnB());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(tested.getNormalOnA());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(tested.getNormalOnB());
      }
   }

   @Test
   public void setToNaN()
   {
      Random random = new Random(641981);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Box3D shapeAOriginal = EuclidShapeRandomTools.nextBox3D(random);
         Box3D shapeA = new Box3D(shapeAOriginal);
         Sphere3D shapeBOriginal = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D shapeB = new Sphere3D(shapeBOriginal);
         EuclidShape3DCollisionResult tested = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         tested.setShapeA(shapeA);
         tested.setShapeB(shapeB);

         tested.setToNaN();

         assertEquals(shapeAOriginal, shapeA);
         assertEquals(shapeBOriginal, shapeB);
         assertFalse(tested.areShapesColliding());
         assertTrue(Double.isNaN(tested.getDistance()));
         assertTrue(Double.isNaN(tested.getSignedDistance()));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(tested.getPointOnA());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(tested.getPointOnB());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(tested.getNormalOnA());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(tested.getNormalOnB());
      }
   }

   @Test
   public void testSwapShapes()
   {
      Random random = new Random(189648);

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult swapped = new EuclidShape3DCollisionResult(original);
         swapped.swapShapes();

         assertEquals(original.getShapeA(), swapped.getShapeB());
         assertEquals(original.getShapeB(), swapped.getShapeA());
         assertEquals(original.areShapesColliding(), swapped.areShapesColliding());
         assertEquals(original.getDistance(), swapped.getDistance());
         assertEquals(original.getSignedDistance(), swapped.getSignedDistance());
         assertEquals(original.getPointOnA(), swapped.getPointOnB());
         assertEquals(original.getPointOnB(), swapped.getPointOnA());
         assertEquals(original.getNormalOnA(), swapped.getNormalOnB());
         assertEquals(original.getNormalOnB(), swapped.getNormalOnA());
      }
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(4841);

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         if (random.nextInt(5) == 0)
            original.getPointOnA().setToNaN();
         if (random.nextInt(5) == 0)
            original.getPointOnB().setToNaN();
         if (random.nextInt(5) == 0)
            original.getNormalOnA().setToNaN();
         if (random.nextInt(5) == 0)
            original.getNormalOnB().setToNaN();

         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         assertTrue(original.epsilonEquals(tested, 0.0));

         tested.setShapeA(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.epsilonEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.epsilonEquals(tested, 0.0));
         tested.setShapeB(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.epsilonEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.epsilonEquals(tested, 0.0));
         tested.setShapesAreColliding(!original.areShapesColliding());
         assertFalse(original.epsilonEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.epsilonEquals(tested, 0.0));
         tested.swapShapes();
         assertFalse(original.epsilonEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         double epsilon = random.nextDouble();
         double sign = random.nextBoolean() ? -1.0 : 1.0;
         tested.setSignedDistance(original.getSignedDistance() + 0.99 * epsilon * sign);
         assertTrue(original.epsilonEquals(tested, epsilon));
         tested.setSignedDistance(original.getSignedDistance() + 1.01 * epsilon * sign);
         assertFalse(original.epsilonEquals(tested, epsilon));

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            if (!tested.getPointOnA().containsNaN())
            {
               tested.set(original);
               assertTrue(original.epsilonEquals(tested, 0.0));
               tested.getPointOnA().setElement(axisIndex, original.getPointOnA().getElement(axisIndex) + 0.99 * epsilon * sign);
               assertTrue(original.epsilonEquals(tested, epsilon));
               tested.getPointOnA().setElement(axisIndex, original.getPointOnA().getElement(axisIndex) + 1.01 * epsilon * sign);
               assertFalse(original.epsilonEquals(tested, epsilon));
            }

            if (!tested.getPointOnB().containsNaN())
            {
               tested.set(original);
               assertTrue(original.epsilonEquals(tested, 0.0));
               tested.getPointOnB().setElement(axisIndex, original.getPointOnB().getElement(axisIndex) + 0.99 * epsilon * sign);
               assertTrue(original.epsilonEquals(tested, epsilon));
               tested.getPointOnB().setElement(axisIndex, original.getPointOnB().getElement(axisIndex) + 1.01 * epsilon * sign);
               assertFalse(original.epsilonEquals(tested, epsilon));
            }

            if (!tested.getNormalOnA().containsNaN())
            {
               tested.set(original);
               assertTrue(original.epsilonEquals(tested, 0.0));
               tested.getNormalOnA().setElement(axisIndex, original.getNormalOnA().getElement(axisIndex) + 0.99 * epsilon * sign);
               assertTrue(original.epsilonEquals(tested, epsilon));
               tested.getNormalOnA().setElement(axisIndex, original.getNormalOnA().getElement(axisIndex) + 1.01 * epsilon * sign);
               assertFalse(original.epsilonEquals(tested, epsilon));
            }

            if (!tested.getNormalOnB().containsNaN())
            {
               tested.set(original);
               assertTrue(original.epsilonEquals(tested, 0.0));
               tested.getNormalOnB().setElement(axisIndex, original.getNormalOnB().getElement(axisIndex) + 0.99 * epsilon * sign);
               assertTrue(original.epsilonEquals(tested, epsilon));
               tested.getNormalOnB().setElement(axisIndex, original.getNormalOnB().getElement(axisIndex) + 1.01 * epsilon * sign);
               assertFalse(original.epsilonEquals(tested, epsilon));
            }
         }
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(4841);

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         if (random.nextInt(5) == 0)
            original.getPointOnA().setToNaN();
         if (random.nextInt(5) == 0)
            original.getPointOnB().setToNaN();
         if (random.nextInt(5) == 0)
            original.getNormalOnA().setToNaN();
         if (random.nextInt(5) == 0)
            original.getNormalOnB().setToNaN();
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         assertTrue(original.geometricallyEquals(tested, 0.0));

         tested.setShapeA(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.geometricallyEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.geometricallyEquals(tested, 0.0));
         tested.setShapeB(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.geometricallyEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.geometricallyEquals(tested, 0.0));
         tested.setShapesAreColliding(!original.areShapesColliding());
         assertFalse(original.geometricallyEquals(tested, Double.POSITIVE_INFINITY));

         tested.set(original);
         assertTrue(original.geometricallyEquals(tested, 0.0));
         tested.swapShapes();
         assertTrue(original.geometricallyEquals(tested, 0.0));

         tested.set(original);
         double epsilon = random.nextDouble();
         double sign = random.nextBoolean() ? -1.0 : 1.0;
         tested.setSignedDistance(original.getSignedDistance() + 0.99 * epsilon * sign);
         if (random.nextBoolean())
            tested.swapShapes();
         assertTrue(original.geometricallyEquals(tested, epsilon));
         tested.set(original);
         if (random.nextBoolean())
            tested.swapShapes();
         tested.setSignedDistance(original.getSignedDistance() + 1.01 * epsilon * sign);
         assertFalse(original.geometricallyEquals(tested, epsilon));

         Vector3D belowTolerance = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.99 * epsilon);
         Vector3D aboveTolerance = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.01 * epsilon);

         if (!original.getPointOnA().containsNaN())
         {
            tested.set(original);
            assertTrue(original.geometricallyEquals(tested, 0.0));
            tested.getPointOnA().add(original.getPointOnA(), belowTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertTrue(original.geometricallyEquals(tested, epsilon));
            tested.set(original);
            tested.getPointOnA().add(original.getPointOnA(), aboveTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertFalse(original.geometricallyEquals(tested, epsilon));
         }

         if (!original.getPointOnB().containsNaN())
         {
            tested.set(original);
            assertTrue(original.geometricallyEquals(tested, 0.0));
            tested.getPointOnB().add(original.getPointOnB(), belowTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertTrue(original.geometricallyEquals(tested, epsilon));
            tested.set(original);
            tested.getPointOnB().add(original.getPointOnB(), aboveTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertFalse(original.geometricallyEquals(tested, epsilon));
         }

         if (!original.getNormalOnA().containsNaN())
         {
            tested.set(original);
            assertTrue(original.geometricallyEquals(tested, 0.0));
            tested.getNormalOnA().add(original.getNormalOnA(), belowTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertTrue(original.geometricallyEquals(tested, epsilon));
            tested.set(original);
            tested.getNormalOnA().add(original.getNormalOnA(), aboveTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertFalse(original.geometricallyEquals(tested, epsilon));
         }

         if (!original.getNormalOnB().containsNaN())
         {
            tested.set(original);
            assertTrue(original.geometricallyEquals(tested, 0.0));
            tested.getNormalOnB().add(original.getNormalOnB(), belowTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertTrue(original.geometricallyEquals(tested, epsilon));
            tested.set(original);
            tested.getNormalOnB().add(original.getNormalOnB(), aboveTolerance);
            if (random.nextBoolean())
               tested.swapShapes();
            assertFalse(original.geometricallyEquals(tested, epsilon));
         }
      }
   }

   @Test
   public void testEquals()
   {
      Random random = new Random(4841);

      for (int i = 0; i < ITERATIONS; i++)
      {
         EuclidShape3DCollisionResult original = EuclidShapeRandomTools.nextEuclidShape3DCollisionResult(random);
         EuclidShape3DCollisionResult tested = new EuclidShape3DCollisionResult(original);
         assertTrue(original.equals(tested));

         tested.setShapeA(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.setShapeB(EuclidShapeRandomTools.nextShape3D(random));
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.setShapesAreColliding(!original.areShapesColliding());
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.swapShapes();
         assertFalse(original.equals(tested));

         tested.set(original);
         double sign = random.nextBoolean() ? -1.0 : 1.0;
         tested.setSignedDistance(original.getSignedDistance() + EPSILON * sign);
         assertFalse(original.equals(tested));

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            tested.set(original);
            assertTrue(original.equals(tested));
            tested.getPointOnA().setElement(axisIndex, original.getPointOnA().getElement(axisIndex) + EPSILON);
            assertFalse(original.equals(tested));

            tested.set(original);
            assertTrue(original.equals(tested));
            tested.getPointOnB().setElement(axisIndex, original.getPointOnB().getElement(axisIndex) + EPSILON);
            assertFalse(original.equals(tested));

            tested.set(original);
            assertTrue(original.equals(tested));
            tested.getNormalOnA().setElement(axisIndex, original.getNormalOnA().getElement(axisIndex) + EPSILON);
            assertFalse(original.equals(tested));

            tested.set(original);
            assertTrue(original.equals(tested));
            tested.getNormalOnB().setElement(axisIndex, original.getNormalOnB().getElement(axisIndex) + EPSILON);
            assertFalse(original.equals(tested));
         }

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.getPointOnA().setToNaN();
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.getPointOnB().setToNaN();
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.getNormalOnA().setToNaN();
         assertFalse(original.equals(tested));

         tested.set(original);
         assertTrue(original.equals(tested));
         tested.getNormalOnB().setToNaN();
         assertFalse(original.equals(tested));
      }
   }
}
