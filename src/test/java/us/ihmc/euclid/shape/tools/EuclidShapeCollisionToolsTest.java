package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.shape.Capsule3D;
import us.ihmc.euclid.shape.CollisionTestResult;
import us.ihmc.euclid.shape.Cylinder3D;
import us.ihmc.euclid.shape.PointShape3D;
import us.ihmc.euclid.shape.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeCollisionToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testPointShape3DBox3D() throws Exception
   {
      Random random = new Random(235425);

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside the Box over one of the faces
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Axis axis = Axis.values[random.nextInt(3)];
         Bound bound = Bound.values()[random.nextInt(2)];
         Point3D pointOnAFace = EuclidShapeRandomTools.nextWeightedAverage(random, getBox3DFaceVertices(axis, bound, box3D));

         Vector3D shiftDirection = new Vector3D(getAxis(axis, box3D.getPose()));
         if (bound == Bound.MIN)
            shiftDirection.negate();

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, shiftDirection, pointOnAFace);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointShape3D);
         expected.getNormalOnA().setAndNegate(shiftDirection);
         expected.getPointOnB().set(pointOnAFace);
         expected.getNormalOnB().set(shiftDirection);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape inside the Box with the closest face known
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Axis axis = Axis.values[random.nextInt(3)];
         Bound bound = Bound.values()[random.nextInt(2)];
         Plane3D closestFacePlane = getBox3DFacePlane(axis, bound, box3D);
         List<Point3D> faceVerticesAndFarthestIn = new ArrayList<>(Arrays.asList(getBox3DFaceVertices(axis, bound, box3D)));

         // Finding how far the point can be inside of the box while the selected face is still the closest.
         // TODO This is a conservative approach and does not consider the entire region of points that are closer the given face. 
         double minHalfSize = 0.5 * EuclidCoreTools.min(box3D.getSizeX(), box3D.getSizeY(), box3D.getSizeZ());
         Point3D farthestInside = new Point3D();
         farthestInside.scaleAdd(-minHalfSize, closestFacePlane.getNormal(), closestFacePlane.getPoint());
         faceVerticesAndFarthestIn.add(farthestInside);
         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, faceVerticesAndFarthestIn);

         Point3D pointOnFace = new Point3D();
         closestFacePlane.orthogonalProjection(pointInside, pointOnFace);

         PointShape3D pointShape3D = new PointShape3D(pointInside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(true);
         expected.setDepth(closestFacePlane.distance(pointInside));
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(closestFacePlane.getNormal());
         expected.getPointOnB().set(pointOnFace);
         expected.getNormalOnB().set(closestFacePlane.getNormal());

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside the Box closest to a randomly picked vertex
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Bound xBound = Bound.values()[random.nextInt(2)];
         Bound yBound = Bound.values()[random.nextInt(2)];
         Bound zBound = Bound.values()[random.nextInt(2)];

         Point3D closestVertex = getVertex(xBound, yBound, zBound, box3D);

         Vector3D xDirection = new Vector3D(box3D.getPose().getXAxis());
         if (xBound == Bound.MIN)
            xDirection.negate();
         Vector3D yDirection = new Vector3D(box3D.getPose().getYAxis());
         if (yBound == Bound.MIN)
            yDirection.negate();
         Vector3D zDirection = new Vector3D(box3D.getPose().getZAxis());
         if (zBound == Bound.MIN)
            zDirection.negate();

         Point3D pointOutside = new Point3D(closestVertex);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), xDirection, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), yDirection, pointOutside);
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), zDirection, pointOutside);

         Vector3D normal = new Vector3D();
         normal.sub(pointOutside, closestVertex);
         normal.normalize();

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(pointOutside.distance(closestVertex));
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(closestVertex);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside the Box closest to a randomly picked edge
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Axis edgeAxis = Axis.values[random.nextInt(3)];
         Axis otherAxis1 = edgeAxis.getNextClockwiseAxis();
         Axis otherAxis2 = otherAxis1.getNextClockwiseAxis();
         double signAxis1 = random.nextBoolean() ? -1.0 : 1.0;
         double signAxis2 = random.nextBoolean() ? -1.0 : 1.0;
         Vector3D direction1 = new Vector3D();
         direction1.setAndScale(signAxis1, getAxis(otherAxis1, box3D.getPose()));
         Vector3D direction2 = new Vector3D();
         direction2.setAndScale(signAxis2, getAxis(otherAxis2, box3D.getPose()));

         Point3D edgeCenter = new Point3D(box3D.getPosition());
         edgeCenter.scaleAdd(0.5 * signAxis1 * otherAxis1.dot(box3D.getSize()), direction1, edgeCenter);
         edgeCenter.scaleAdd(0.5 * signAxis2 * otherAxis2.dot(box3D.getSize()), direction2, edgeCenter);

         Point3D pointOnEdge = new Point3D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.5);
         pointOnEdge.scaleAdd(alpha * edgeAxis.dot(box3D.getSize()), getAxis(edgeAxis, box3D.getPose()), edgeCenter);

         Point3D pointOutside = new Point3D(pointOnEdge);
         pointOutside.scaleAdd(signAxis1 * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), direction1, pointOutside);
         pointOutside.scaleAdd(signAxis2 * EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), direction2, pointOutside);

         Vector3D normal = new Vector3D();
         normal.sub(pointOutside, pointOnEdge);
         normal.normalize();

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(pointOutside.distance(pointOnEdge));
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnEdge);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPointShape3DCapsule3D() throws Exception
   {
      Random random = new Random(5411);

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside closest to one of the capsule's ends.
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3DReadOnly capsuleAxis = capsule3D.getAxis();

         // We'll choose which end is closer with this sign times the capsule axis
         double endSign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D endCenter = new Point3D();
         endCenter.scaleAdd(0.5 * endSign * capsule3D.getLength(), capsuleAxis, capsule3D.getPosition());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsuleAxis, false);

         Vector3D normal = new Vector3D();
         normal.setAndScale(endSign, capsuleAxis);
         normal.interpolate(orthogonalToAxis, random.nextDouble());
         normal.normalize();

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(capsule3D.getRadius(), normal, endCenter);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnSurface);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside closest to the cylinder part of the capsule.
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3DReadOnly capsuleAxis = capsule3D.getAxis();

         Point3D pointOnAxis = new Point3D();
         pointOnAxis.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.5 * capsule3D.getLength()), capsuleAxis, capsule3D.getPosition());

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsuleAxis, true);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(capsule3D.getRadius(), normal, pointOnAxis);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnSurface);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape inside closest to one of the capsule's ends.
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3DReadOnly capsuleAxis = capsule3D.getAxis();

         // We'll choose which end is closer with this sign times the capsule axis
         double endSign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D endCenter = new Point3D();
         endCenter.scaleAdd(0.5 * endSign * capsule3D.getLength(), capsuleAxis, capsule3D.getPosition());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsuleAxis, false);

         Vector3D normal = new Vector3D();
         normal.setAndScale(endSign, capsuleAxis);
         normal.interpolate(orthogonalToAxis, random.nextDouble());
         normal.normalize();

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(capsule3D.getRadius(), normal, endCenter);

         Point3D pointInside = new Point3D();
         pointInside.interpolate(endCenter, pointOnSurface, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(true);
         expected.setDepth(distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside closest to the cylinder part of the capsule.
         Capsule3D capsule3D = EuclidShapeRandomTools.nextCapsule3D(random);
         Vector3DReadOnly capsuleAxis = capsule3D.getAxis();

         Point3D pointOnAxis = new Point3D();
         pointOnAxis.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.5 * capsule3D.getLength()), capsuleAxis, capsule3D.getPosition());

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, capsuleAxis, true);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(capsule3D.getRadius(), normal, pointOnAxis);

         Point3D pointInside = new Point3D();
         pointInside.interpolate(pointOnAxis, pointOnSurface, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         double distance = pointInside.distance(pointOnSurface);

         PointShape3D pointShape3D = new PointShape3D(pointInside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(true);
         expected.setDepth(distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }
   }

   @Test
   public void testPointShape3DCylinder3D() throws Exception
   {
      Random random = new Random(42352);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to one of the cylinder's ends.
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3DReadOnly cylinderAxis = cylinder3D.getAxis();

         // We'll choose which end is closer with this sign times the capsule axis
         double endSign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D endCenter = new Point3D();
         endCenter.scaleAdd(0.5 * endSign * cylinder3D.getLength(), cylinderAxis, cylinder3D.getPosition());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinderAxis, true);

         Point3D pointOnSurface = new Point3D();
         double distanceFromCenter = EuclidCoreRandomTools.nextDouble(random, 0.0, cylinder3D.getRadius());
         pointOnSurface.scaleAdd(distanceFromCenter, orthogonalToAxis, endCenter);

         Vector3D normal = new Vector3D();
         normal.setAndScale(endSign, cylinderAxis);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnSurface);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the cylinder side.
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3DReadOnly cylinderAxis = cylinder3D.getAxis();

         Point3D pointOnAxis = new Point3D();
         pointOnAxis.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.5 * cylinder3D.getLength()), cylinderAxis, cylinder3D.getPosition());

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinderAxis, true);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(cylinder3D.getRadius(), normal, pointOnAxis);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnSurface);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside closest to the edge of one of the cylinder's ends.
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3DReadOnly cylinderAxis = cylinder3D.getAxis();

         // We'll choose which end is closer with this sign times the capsule axis
         double endSign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D endCenter = new Point3D();
         endCenter.scaleAdd(0.5 * endSign * cylinder3D.getLength(), cylinderAxis, cylinder3D.getPosition());

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinderAxis, true);

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, endCenter);

         Vector3D normal = new Vector3D();
         normal.setAndScale(endSign, cylinderAxis);
         normal.interpolate(orthogonalToAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnEdge);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnEdge);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside closest to one of the cylinder's ends.
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3DReadOnly cylinderAxis = cylinder3D.getAxis();

         // We'll choose which end is closer with this sign times the capsule axis
         double endSign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D endCenter = new Point3D();
         endCenter.scaleAdd(0.5 * endSign * cylinder3D.getLength(), cylinderAxis, cylinder3D.getPosition());

         Vector3D normal = new Vector3D();
         normal.setAndScale(endSign, cylinderAxis);

         Plane3D endPlane = new Plane3D(endCenter, normal);

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinderAxis, true);

         List<Point3D> regionToExplore = new ArrayList<>();

         Point3D pointOnEdge = new Point3D();
         pointOnEdge.scaleAdd(cylinder3D.getRadius(), orthogonalToAxis, endCenter);
         Point3D pointOnEdgeOpposideSide = new Point3D();
         pointOnEdgeOpposideSide.scaleAdd(-cylinder3D.getRadius(), orthogonalToAxis, endCenter);
         regionToExplore.add(pointOnEdge);
         regionToExplore.add(pointOnEdgeOpposideSide);

         if (cylinder3D.getRadius() < 0.5 * cylinder3D.getLength())
         {
            Point3D farthestPointInside = new Point3D();
            double maxDistanceIn = cylinder3D.getRadius();
            farthestPointInside.scaleAdd(-endSign * maxDistanceIn, cylinderAxis, endCenter);
            regionToExplore.add(farthestPointInside);
         }
         else
         { // The region that is the farthest from the end form a circle, beyond it the points are closer to the opposite end. 
            Point3D farthestInsideCenter = new Point3D();
            double maxDistanceIn = 0.5 * cylinder3D.getLength();
            farthestInsideCenter.scaleAdd(-endSign * maxDistanceIn, cylinderAxis, endCenter);

            double radius = cylinder3D.getRadius() - 0.5 * cylinder3D.getLength();
            Point3D farthestInside1 = new Point3D();
            Point3D farthestInside2 = new Point3D();
            farthestInside1.scaleAdd(radius, orthogonalToAxis, farthestInsideCenter);
            farthestInside2.scaleAdd(-radius, orthogonalToAxis, farthestInsideCenter);
            regionToExplore.add(farthestInside1);
            regionToExplore.add(farthestInside2);
         }

         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, regionToExplore);

         Point3D pointOnSurface = new Point3D();
         endPlane.orthogonalProjection(pointInside, pointOnSurface);
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(true);
         expected.setDepth(distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point inside closest to the cylinder's side.
         Cylinder3D cylinder3D = EuclidShapeRandomTools.nextCylinder3D(random);
         Vector3DReadOnly cylinderAxis = cylinder3D.getAxis();

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, cylinderAxis, true);
         Point3D centerOnSurface = new Point3D();
         centerOnSurface.scaleAdd(cylinder3D.getRadius(), normal, cylinder3D.getPosition());

         List<Point3D> regionToExplore = new ArrayList<>();

         Point3D pointOnOneEnd = new Point3D();
         pointOnOneEnd.scaleAdd(0.5 * cylinder3D.getLength(), cylinderAxis, centerOnSurface);
         Point3D pointOnEdgeOtherEnd = new Point3D();
         pointOnEdgeOtherEnd.scaleAdd(-0.5 * cylinder3D.getLength(), cylinderAxis, centerOnSurface);
         regionToExplore.add(pointOnOneEnd);
         regionToExplore.add(pointOnEdgeOtherEnd);

         if (cylinder3D.getRadius() < 0.5 * cylinder3D.getLength())
         {
            double positionFromCenter = 0.5 * cylinder3D.getLength() - cylinder3D.getRadius();
            Point3D farthestInside1 = new Point3D();
            Point3D farthestInside2 = new Point3D();
            farthestInside1.scaleAdd(positionFromCenter, cylinderAxis, cylinder3D.getPosition());
            farthestInside2.scaleAdd(-positionFromCenter, cylinderAxis, cylinder3D.getPosition());
            regionToExplore.add(farthestInside1);
            regionToExplore.add(farthestInside2);
         }
         else
         {
            Point3D farthestInside = new Point3D();
            double minDistanceFromAxis = cylinder3D.getRadius() - 0.5 * cylinder3D.getLength();
            farthestInside.scaleAdd(minDistanceFromAxis, normal, cylinder3D.getPosition());
            regionToExplore.add(farthestInside);
         }

         Point3D pointInside = EuclidShapeRandomTools.nextWeightedAverage(random, regionToExplore);

         Point3D pointOnSurface = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointInside, centerOnSurface, normal);
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);
         pointShape3D.getOrientation().set(EuclidCoreRandomTools.nextRotationMatrix(random)); // Just to verify that the orientation does not change anything

         CollisionTestResult expected = new CollisionTestResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(true);
         expected.setDepth(distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         CollisionTestResult actual = new CollisionTestResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertCollisionTestResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }
   }

   public static Vector3DReadOnly getAxis(Axis axis, Shape3DPoseReadOnly shape3DPose)
   {
      switch (axis)
      {
      case X:
         return shape3DPose.getXAxis();
      case Y:
         return shape3DPose.getYAxis();
      case Z:
         return shape3DPose.getZAxis();
      default:
         throw new RuntimeException("Unknown axis: " + axis);
      }
   }

   public static Point3D getVertex(Bound xBound, Bound yBound, Bound zBound, Box3DReadOnly box3D)
   {
      double xShift = 0.5 * (xBound == Bound.MAX ? 1.0 : -1.0) * box3D.getSizeX();
      double yShift = 0.5 * (yBound == Bound.MAX ? 1.0 : -1.0) * box3D.getSizeY();
      double zShift = 0.5 * (zBound == Bound.MAX ? 1.0 : -1.0) * box3D.getSizeZ();

      Point3D vertex = new Point3D(box3D.getPosition());
      vertex.scaleAdd(xShift, box3D.getPose().getXAxis(), vertex);
      vertex.scaleAdd(yShift, box3D.getPose().getYAxis(), vertex);
      vertex.scaleAdd(zShift, box3D.getPose().getZAxis(), vertex);
      return vertex;
   }

   public static Plane3D getBox3DFacePlane(Axis axis, Bound bound, Box3DReadOnly box3D)
   {
      Plane3D facePlane = new Plane3D();

      Vector3D faceNormal = new Vector3D(axis);
      if (bound == Bound.MIN)
         faceNormal.negate();

      Point3D faceCenter = new Point3D();
      faceCenter.scaleAdd(faceNormal.dot(box3D.getSize()), axis, faceCenter);

      faceCenter.scale(0.5);
      facePlane.set(faceCenter, faceNormal);
      facePlane.applyTransform(box3D.getPose());

      return facePlane;
   }

   public static Point3D[] getBox3DFaceVertices(Axis axis, Bound bound, Box3DReadOnly box3D)
   {
      Point3D[] faceVertices = new Point3D[4];
      Axis otherAxis1 = axis.getNextClockwiseAxis();
      Axis otherAxis2 = otherAxis1.getNextClockwiseAxis();

      for (int i = 0; i < 4; i++)
      {
         Point3D vertex = new Point3D();
         if (bound == Bound.MAX)
            vertex.scaleAdd(axis.dot(box3D.getSize()), axis, vertex);
         else
            vertex.scaleAdd(-axis.dot(box3D.getSize()), axis, vertex);

         if ((i & 1) == 0)
            vertex.scaleAdd(otherAxis1.dot(box3D.getSize()), otherAxis1, vertex);
         else
            vertex.scaleAdd(-otherAxis1.dot(box3D.getSize()), otherAxis1, vertex);

         if ((i & 2) == 0)
            vertex.scaleAdd(otherAxis2.dot(box3D.getSize()), otherAxis2, vertex);
         else
            vertex.scaleAdd(-otherAxis2.dot(box3D.getSize()), otherAxis2, vertex);

         vertex.scale(0.5);
         box3D.getPose().transform(vertex);
         faceVertices[i] = vertex;
      }

      return faceVertices;
   }
}
