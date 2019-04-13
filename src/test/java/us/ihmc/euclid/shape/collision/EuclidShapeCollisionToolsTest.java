package us.ihmc.euclid.shape.collision;

import static us.ihmc.euclid.EuclidTestConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.tools.EuclidEllipsoid3DTools;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/*
 * TODO Need to test the edge-cases, for instance when a PointShape3D is at the center of the other
 * shape.
 */
public class EuclidShapeCollisionToolsTest
{
   private static final double EPSILON = 5.0e-11;

   @Test
   public void testPointShape3DBox3D() throws Exception
   {
      Random random = new Random(235425);

      for (int i = 0; i < ITERATIONS; i++)
      { // PointShape outside the Box over one of the faces
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         Axis axis = Axis.values[random.nextInt(3)];
         Bound bound = Bound.values()[random.nextInt(2)];
         Point3D pointOnAFace = EuclidGeometryRandomTools.nextWeightedAverage(random, getBox3DFaceVertices(axis, bound, box3D));

         Vector3D shiftDirection = new Vector3D(getAxis(axis, box3D.getPose()));
         if (bound == Bound.MIN)
            shiftDirection.negate();

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, shiftDirection, pointOnAFace);

         PointShape3D pointShape3D = new PointShape3D(pointOutside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointShape3D);
         expected.getNormalOnA().setAndNegate(shiftDirection);
         expected.getPointOnB().set(pointOnAFace);
         expected.getNormalOnB().set(shiftDirection);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, EPSILON);
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
         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, faceVerticesAndFarthestIn);

         Point3D pointOnFace = new Point3D();
         closestFacePlane.orthogonalProjection(pointInside, pointOnFace);

         PointShape3D pointShape3D = new PointShape3D(pointInside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-closestFacePlane.distance(pointInside));
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(closestFacePlane.getNormal());
         expected.getPointOnB().set(pointOnFace);
         expected.getNormalOnB().set(closestFacePlane.getNormal());

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(pointOutside.distance(closestVertex));
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(closestVertex);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(box3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(pointOutside.distance(pointOnEdge));
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnEdge);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DBox3DCollisionTest(pointShape3D, box3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals(expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(capsule3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCapsule3DCollisionTest(pointShape3D, capsule3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnEdge);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, regionToExplore);

         Point3D pointOnSurface = new Point3D();
         endPlane.orthogonalProjection(pointInside, pointOnSurface);
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, regionToExplore);

         Point3D pointOnSurface = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointInside, centerOnSurface, normal);
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(cylinder3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DCylinder3DCollisionTest(pointShape3D, cylinder3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }
   }

   @Test
   public void testPointShape3DEllipsoid3D() throws Exception
   {
      Random random = new Random(54652);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial case: point laying on one of the ellipsoid principal axes
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.01, 1.0);

         Axis localAxis = Axis.values[random.nextInt(3)];
         double sign = random.nextBoolean() ? -1.0 : 1.0;
         double radius = localAxis.dot(ellipsoid3D.getRadii());
         Vector3D normal = new Vector3D();
         normal.setAndScale(sign, getAxis(localAxis, ellipsoid3D.getPose()));

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(radius, normal, ellipsoid3D.getPosition());

         { // Do test with point being outside
            double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(distance, normal, pointOnSurface);

            PointShape3D pointShape3D = new PointShape3D(pointOutside);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(ellipsoid3D);
            expected.setShapesAreColliding(false);
            expected.setDistance(distance);
            expected.getPointOnA().set(pointOutside);
            expected.getNormalOnA().setAndNegate(normal);
            expected.getPointOnB().set(pointOnSurface);
            expected.getNormalOnB().set(normal);

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }

         { // Do test with point being inside
            Point3D pointInsideWorld = new Point3D();
            double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.9995); // When the point gets too close to the center, it triggers an edge-case.
            pointInsideWorld.interpolate(pointOnSurface, ellipsoid3D.getPosition(), alpha);
            Point3D pointInsideLocal = new Point3D(pointInsideWorld);
            ellipsoid3D.transformToLocal(pointInsideLocal);

            PointShape3D pointShape3D = new PointShape3D(pointInsideWorld);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(ellipsoid3D);
            expected.setShapesAreColliding(true);
            expected.setDistance(EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointInsideLocal, expected.getPointOnB()));
            ellipsoid3D.transformToWorld(expected.getPointOnB());
            expected.getPointOnA().set(pointInsideWorld);
            expected.getNormalOnB().sub(expected.getPointOnB(), pointInsideWorld);
            expected.getNormalOnB().normalize();
            expected.getNormalOnA().setAndNegate(expected.getNormalOnB());

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point outside. Using the scale trick to place the point on the surface.
         Ellipsoid3D ellipsoid3D = EuclidShapeRandomTools.nextEllipsoid3D(random, 0.1, 1.0);

         // Build the configuration assuming a unit-sphere with its pose set to identity
         Vector3D unitVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Point3D pointOnSurface = new Point3D(unitVector);
         Vector3D normal = new Vector3D(unitVector);
         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), normal, pointOnSurface);
         Point3D pointInside = new Point3D();
         pointInside.interpolate(pointOnSurface, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Go in the ellipsoid space by scaling everything with its radii
         pointOnSurface.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         normal.scale(1.0 / ellipsoid3D.getRadiusX(), 1.0 / ellipsoid3D.getRadiusY(), 1.0 / ellipsoid3D.getRadiusZ());
         normal.normalize();
         pointOutside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());
         pointInside.scale(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ());

         // Apply to the pose of the ellipsoid.
         pointOnSurface.applyTransform(ellipsoid3D.getPose());
         normal.applyTransform(ellipsoid3D.getPose());
         pointOutside.applyTransform(ellipsoid3D.getPose());
         pointInside.applyTransform(ellipsoid3D.getPose());

         { // Do test with point being outside
            PointShape3D pointShape3D = new PointShape3D(pointOutside);
            Point3D pointOutsideLocal = new Point3D(pointOutside);
            ellipsoid3D.transformToLocal(pointOutsideLocal);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(ellipsoid3D);
            expected.setShapesAreColliding(false);
            expected.setDistance(EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointOutsideLocal, expected.getPointOnB()));
            ellipsoid3D.transformToWorld(expected.getPointOnB());
            expected.getPointOnA().set(pointOutside);
            expected.getNormalOnB().sub(expected.getPointOnA(), expected.getPointOnB());
            expected.getNormalOnB().normalize();
            expected.getNormalOnA().setAndNegate(expected.getNormalOnB());

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }

         { // Do test with point being inside
            PointShape3D pointShape3D = new PointShape3D(pointInside);
            Point3D pointInsideLocal = new Point3D(pointInside);
            ellipsoid3D.transformToLocal(pointInsideLocal);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(ellipsoid3D);
            expected.setShapesAreColliding(true);
            expected.setDistance(EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(ellipsoid3D.getRadii(), pointInsideLocal, expected.getPointOnB()));
            ellipsoid3D.transformToWorld(expected.getPointOnB());
            expected.getPointOnA().set(pointInside);
            expected.getNormalOnB().sub(expected.getPointOnB(), expected.getPointOnA());
            expected.getNormalOnB().normalize();
            expected.getNormalOnA().setAndNegate(expected.getNormalOnB());

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testPointShape3DRamp3D() throws Exception
   {
      Random random = new Random(346536);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, below the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOnSurface.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         ramp3D.getPose().transform(pointOnSurface);

         Vector3D normal = new Vector3D();
         normal.setAndNegate(ramp3D.getPose().getZAxis());

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnSurface, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, beyond the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.setX(ramp3D.getSizeX());
         pointOnSurface.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOnSurface.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.getPose().transform(pointOnSurface);

         Vector3D normal = new Vector3D();
         normal.set(ramp3D.getPose().getXAxis());

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnSurface, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, above the slope face of the ramp
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnSurface = new Point3D();
         double distanceOnSlope = EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getRampLength());
         pointOnSurface.setX(distanceOnSlope * Math.cos(ramp3D.getRampIncline()));
         pointOnSurface.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOnSurface.setZ(distanceOnSlope * Math.sin(ramp3D.getRampIncline()));
         ramp3D.getPose().transform(pointOnSurface);

         Vector3D normal = new Vector3D();
         ramp3D.getRampSurfaceNormal(normal);

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnSurface, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, in front of the ramp, closest to the edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnFrontEdge = new Point3D();
         pointOnFrontEdge.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         ramp3D.getPose().transform(pointOnFrontEdge);

         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(rampNormal, bottomNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnFrontEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, below of the ramp, closest to the left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnLeftEdge = new Point3D();
         pointOnLeftEdge.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOnLeftEdge.setY(0.5 * ramp3D.getSizeY());
         ramp3D.getPose().transform(pointOnLeftEdge);

         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(leftNormal, bottomNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnLeftEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, below of the ramp, closest to the right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnRightEdge = new Point3D();
         pointOnRightEdge.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeX()));
         pointOnRightEdge.setY(-0.5 * ramp3D.getSizeY());
         ramp3D.getPose().transform(pointOnRightEdge);

         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(rightNormal, bottomNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnRightEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, below of the ramp, closest to the rear edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnRearEdge = new Point3D();
         pointOnRearEdge.setX(ramp3D.getSizeX());
         pointOnRearEdge.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         ramp3D.getPose().transform(pointOnRearEdge);

         Vector3D backNormal = new Vector3D();
         backNormal.set(ramp3D.getPose().getXAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(backNormal, bottomNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnRearEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, beyond of the ramp, closest to the left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnLeftEdge = new Point3D();
         pointOnLeftEdge.setX(ramp3D.getSizeX());
         pointOnLeftEdge.setY(0.5 * ramp3D.getSizeY());
         pointOnLeftEdge.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.getPose().transform(pointOnLeftEdge);

         Vector3D backNormal = new Vector3D();
         backNormal.set(ramp3D.getPose().getXAxis());
         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(backNormal, leftNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnLeftEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, beyond of the ramp, closest to the right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnRightEdge = new Point3D();
         pointOnRightEdge.setX(ramp3D.getSizeX());
         pointOnRightEdge.setY(-0.5 * ramp3D.getSizeY());
         pointOnRightEdge.setZ(EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getSizeZ()));
         ramp3D.getPose().transform(pointOnRightEdge);

         Vector3D backNormal = new Vector3D();
         backNormal.set(ramp3D.getPose().getXAxis());
         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(backNormal, rightNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnRightEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the top edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnTopEdge = new Point3D();
         pointOnTopEdge.setX(ramp3D.getSizeX());
         pointOnTopEdge.setY(EuclidCoreRandomTools.nextDouble(random, 0.5 * ramp3D.getSizeY()));
         pointOnTopEdge.setZ(ramp3D.getSizeZ());
         ramp3D.getPose().transform(pointOnTopEdge);

         Vector3D backNormal = new Vector3D();
         backNormal.set(ramp3D.getPose().getXAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(backNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnTopEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, above the ramp, closest to the left edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnLeftEdge = new Point3D();
         double distanceOnSlope = EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getRampLength());
         pointOnLeftEdge.setX(distanceOnSlope * Math.cos(ramp3D.getRampIncline()));
         pointOnLeftEdge.setY(0.5 * ramp3D.getSizeY());
         pointOnLeftEdge.setZ(distanceOnSlope * Math.sin(ramp3D.getRampIncline()));
         ramp3D.getPose().transform(pointOnLeftEdge);

         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(leftNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnLeftEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, above the ramp, closest to the right edge
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D pointOnLeftEdge = new Point3D();
         double distanceOnSlope = EuclidCoreRandomTools.nextDouble(random, 0.0, ramp3D.getRampLength());
         pointOnLeftEdge.setX(distanceOnSlope * Math.cos(ramp3D.getRampIncline()));
         pointOnLeftEdge.setY(-0.5 * ramp3D.getSizeY());
         pointOnLeftEdge.setZ(distanceOnSlope * Math.sin(ramp3D.getRampIncline()));
         ramp3D.getPose().transform(pointOnLeftEdge);

         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(rightNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, pointOnLeftEdge, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the front-left vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontLeftVertex = new Point3D();
         frontLeftVertex.setX(0.0);
         frontLeftVertex.setY(0.5 * ramp3D.getSizeY());
         frontLeftVertex.setZ(0.0);
         ramp3D.getPose().transform(frontLeftVertex);

         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(bottomNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(leftNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontLeftVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the front-right vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontLeftVertex = new Point3D();
         frontLeftVertex.setX(0.0);
         frontLeftVertex.setY(-0.5 * ramp3D.getSizeY());
         frontLeftVertex.setZ(0.0);
         ramp3D.getPose().transform(frontLeftVertex);

         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(bottomNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(rightNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontLeftVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the top-left vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontLeftVertex = new Point3D();
         frontLeftVertex.setX(ramp3D.getSizeX());
         frontLeftVertex.setY(0.5 * ramp3D.getSizeY());
         frontLeftVertex.setZ(ramp3D.getSizeZ());
         ramp3D.getPose().transform(frontLeftVertex);

         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());
         Vector3D rearNormal = new Vector3D();
         rearNormal.set(ramp3D.getPose().getXAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(rearNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(leftNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontLeftVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the top-right vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontRightVertex = new Point3D();
         frontRightVertex.setX(ramp3D.getSizeX());
         frontRightVertex.setY(-0.5 * ramp3D.getSizeY());
         frontRightVertex.setZ(ramp3D.getSizeZ());
         ramp3D.getPose().transform(frontRightVertex);

         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D rearNormal = new Vector3D();
         rearNormal.set(ramp3D.getPose().getXAxis());
         Vector3D rampNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(rampNormal);

         Vector3D normal = new Vector3D();
         normal.interpolate(rearNormal, rampNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(rightNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontRightVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the bottom-left vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontRightVertex = new Point3D();
         frontRightVertex.setX(ramp3D.getSizeX());
         frontRightVertex.setY(0.5 * ramp3D.getSizeY());
         ramp3D.getPose().transform(frontRightVertex);

         Vector3D rearNormal = new Vector3D();
         rearNormal.set(ramp3D.getPose().getXAxis());
         Vector3D leftNormal = new Vector3D();
         leftNormal.set(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(bottomNormal, rearNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(leftNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontRightVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside, closest to the bottom-right vertex
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         Point3D frontRightVertex = new Point3D();
         frontRightVertex.setX(ramp3D.getSizeX());
         frontRightVertex.setY(-0.5 * ramp3D.getSizeY());
         ramp3D.getPose().transform(frontRightVertex);

         Vector3D rearNormal = new Vector3D();
         rearNormal.set(ramp3D.getPose().getXAxis());
         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());
         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());

         Vector3D normal = new Vector3D();
         normal.interpolate(bottomNormal, rearNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();
         normal.interpolate(rightNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         normal.normalize();

         buildPointOutsideAndPerformAssertion(random, i, ramp3D, frontRightVertex, normal);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape inside ramp. Instead of manually writing the different scenarios, we'll use planes to identify which face is closest.
         Ramp3D ramp3D = EuclidShapeRandomTools.nextRamp3D(random);
         double halfWidth = 0.5 * ramp3D.getSizeY();

         Point3D bottomPoint = new Point3D(0.0, 0.0, 0.0);
         Point3D rearPoint = new Point3D(ramp3D.getSizeX(), 0.0, 0.0);
         Point3D slopePoint = new Point3D(0.0, 0.0, 0.0);
         Point3D leftPoint = new Point3D(0.0, halfWidth, 0.0);
         Point3D rightPoint = new Point3D(0.0, -halfWidth, 0.0);
         Arrays.asList(bottomPoint, rearPoint, slopePoint, leftPoint, rightPoint).forEach(ramp3D.getPose()::transform);

         Vector3D bottomNormal = new Vector3D();
         bottomNormal.setAndNegate(ramp3D.getPose().getZAxis());
         Vector3D rearNormal = new Vector3D(ramp3D.getPose().getXAxis());
         Vector3D slopeNormal = new Vector3D();
         ramp3D.getRampSurfaceNormal(slopeNormal);
         Vector3D leftNormal = new Vector3D(ramp3D.getPose().getYAxis());
         Vector3D rightNormal = new Vector3D();
         rightNormal.setAndNegate(ramp3D.getPose().getYAxis());

         Plane3D bottomFace = new Plane3D(bottomPoint, bottomNormal);
         Plane3D rearFace = new Plane3D(rearPoint, rearNormal);
         Plane3D slopeFace = new Plane3D(slopePoint, slopeNormal);
         Plane3D leftFace = new Plane3D(leftPoint, leftNormal);
         Plane3D rightFace = new Plane3D(rightPoint, rightNormal);
         List<Plane3D> faces = Arrays.asList(bottomFace, rearFace, slopeFace, leftFace, rightFace);

         List<Point3D> vertices = Arrays.asList(new Point3D(0.0, -halfWidth, 0.0), new Point3D(0.0, halfWidth, 0.0),
                                                new Point3D(ramp3D.getSizeX(), -halfWidth, 0.0), new Point3D(ramp3D.getSizeX(), halfWidth, 0.0),
                                                new Point3D(ramp3D.getSizeX(), -halfWidth, ramp3D.getSizeZ()),
                                                new Point3D(ramp3D.getSizeX(), halfWidth, ramp3D.getSizeZ()));
         vertices.forEach(ramp3D.getPose()::transform);

         Point3D pointInside = EuclidGeometryRandomTools.nextWeightedAverage(random, vertices);

         Plane3D closestFace = faces.stream().sorted((a, b) -> Double.compare(a.distance(pointInside), b.distance(pointInside))).findFirst().get();
         Point3D pointOnSurface = closestFace.orthogonalProjectionCopy(pointInside);
         double depth = closestFace.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(ramp3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-depth);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(closestFace.getNormal());
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(closestFace.getNormal());

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DRamp3DCollisionTest(pointShape3D, ramp3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }
   }

   private static void buildPointOutsideAndPerformAssertion(Random random, int iteration, Ramp3DReadOnly ramp3D, Point3DReadOnly pointOnShape,
                                                            Vector3DReadOnly normal)
   {
      double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
      Point3D pointOutside = new Point3D();
      pointOutside.scaleAdd(distance, normal, pointOnShape);

      PointShape3D pointShape3D = new PointShape3D(pointOutside);

      EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
      expected.setToNaN();
      expected.setShapeA(pointShape3D);
      expected.setShapeB(ramp3D);
      expected.setShapesAreColliding(false);
      expected.setDistance(distance);
      expected.getPointOnA().set(pointOutside);
      expected.getNormalOnA().setAndNegate(normal);
      expected.getPointOnB().set(pointOnShape);
      expected.getNormalOnB().set(normal);

      EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
      EuclidShapeCollisionTools.doPointShape3DRamp3DCollisionTest(pointShape3D, ramp3D, actual);
      EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + iteration + "\n", expected, actual, EPSILON);
   }

   @Test
   public void testPointShape3DSphere3D() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Sphere3D sphere3D = EuclidShapeRandomTools.nextSphere3D(random);

         Vector3D normal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(sphere3D.getRadius(), normal, sphere3D.getPosition());

         { // Do test with point being outside
            double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
            Point3D pointOutside = new Point3D();
            pointOutside.scaleAdd(distance, normal, pointOnSurface);

            PointShape3D pointShape3D = new PointShape3D(pointOutside);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(sphere3D);
            expected.setShapesAreColliding(false);
            expected.setDistance(distance);
            expected.getPointOnA().set(pointOutside);
            expected.getNormalOnA().setAndNegate(normal);
            expected.getPointOnB().set(pointOnSurface);
            expected.getNormalOnB().set(normal);

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DSphere3DCollisionTest(pointShape3D, sphere3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }

         { // Do test with point being inside
            Point3D pointInside = new Point3D();
            double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 0.9995); // When the point gets too close to the center, it triggers an edge-case.
            pointInside.interpolate(pointOnSurface, sphere3D.getPosition(), alpha);
            double distance = pointInside.distance(pointOnSurface);

            PointShape3D pointShape3D = new PointShape3D(pointInside);

            EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
            expected.setToNaN();
            expected.setShapeA(pointShape3D);
            expected.setShapeB(sphere3D);
            expected.setShapesAreColliding(true);
            expected.setDistance(-distance);
            expected.getPointOnA().set(pointInside);
            expected.getNormalOnA().setAndNegate(normal);
            expected.getPointOnB().set(pointOnSurface);
            expected.getNormalOnB().set(normal);

            EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
            EuclidShapeCollisionTools.doPointShape3DSphere3DCollisionTest(pointShape3D, sphere3D, actual);
            EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
         }
      }
   }

   @Test
   public void testPointShape3DTorus3D() throws Exception
   {
      Random random = new Random(45645);

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape outside the torus
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         Vector3DReadOnly torusAxis = torus3D.getAxis();

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torusAxis, true);

         Point3D pointOnTubeAxis = new Point3D();
         pointOnTubeAxis.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeDirection = new Vector3D();
         tubeDirection.cross(torusAxis, orthogonalToAxis);
         if (random.nextBoolean())
            tubeDirection.negate();

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeDirection, true);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(torus3D.getTubeRadius(), normal, pointOnTubeAxis);
         double distance;

         if (normal.dot(orthogonalToAxis) < 0.0)
         { // The point-on-surface is in the inner part of the tube, need to be careful when placing the point outside.
            double angle = orthogonalToAxis.angle(normal);
            double maxDistance = torus3D.getRadius() / Math.cos(angle) - torus3D.getTubeRadius();
            if (maxDistance < 0.0)
            { // The torus is likely to have no inner empty space, resetting this iteration
               i--;
               continue;
            }

            distance = EuclidCoreRandomTools.nextDouble(random, 0.0, maxDistance);
         }
         else
         {
            distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         }

         Point3D pointOutside = new Point3D();
         pointOutside.scaleAdd(distance, normal, pointOnSurface);
         PointShape3D pointShape3D = new PointShape3D(pointOutside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(torus3D);
         expected.setShapesAreColliding(false);
         expected.setDistance(distance);
         expected.getPointOnA().set(pointOutside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DTorus3DCollisionTest(pointShape3D, torus3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Point shape inside the torus
         Torus3D torus3D = EuclidShapeRandomTools.nextTorus3D(random);
         Vector3DReadOnly torusAxis = torus3D.getAxis();

         Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, torusAxis, true);

         Point3D pointOnTubeAxis = new Point3D();
         pointOnTubeAxis.scaleAdd(torus3D.getRadius(), orthogonalToAxis, torus3D.getPosition());

         Vector3D tubeDirection = new Vector3D();
         tubeDirection.cross(torusAxis, orthogonalToAxis);
         if (random.nextBoolean())
            tubeDirection.negate();

         Vector3D normal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, tubeDirection, true);

         Point3D pointOnSurface = new Point3D();
         pointOnSurface.scaleAdd(torus3D.getTubeRadius(), normal, pointOnTubeAxis);
         Point3D pointInside = new Point3D();
         pointInside.interpolate(pointOnSurface, pointOnTubeAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         double distance = pointOnSurface.distance(pointInside);

         PointShape3D pointShape3D = new PointShape3D(pointInside);

         EuclidShape3DCollisionResult expected = new EuclidShape3DCollisionResult();
         expected.setToNaN();
         expected.setShapeA(pointShape3D);
         expected.setShapeB(torus3D);
         expected.setShapesAreColliding(true);
         expected.setDistance(-distance);
         expected.getPointOnA().set(pointInside);
         expected.getNormalOnA().setAndNegate(normal);
         expected.getPointOnB().set(pointOnSurface);
         expected.getNormalOnB().set(normal);

         EuclidShape3DCollisionResult actual = new EuclidShape3DCollisionResult();
         EuclidShapeCollisionTools.doPointShape3DTorus3DCollisionTest(pointShape3D, torus3D, actual);
         EuclidShapeTestTools.assertEuclidShape3DCollisionResultEquals("Iteration: " + i + "\n", expected, actual, EPSILON);
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
