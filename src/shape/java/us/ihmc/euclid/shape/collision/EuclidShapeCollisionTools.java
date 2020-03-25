package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * This class provides tools to evaluate collision between primitive shapes.
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeCollisionTools
{
   private EuclidShapeCollisionTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Evaluates the collision state between a point shape and a box.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param box3D        the box. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DBox3DCollision(PointShape3DReadOnly pointShape3D, Box3DReadOnly box3D,
                                                         EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DBox3DCollision(pointShape3D, box3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(box3D);
   }

   /**
    * Evaluates the collision state between a sphere and a box.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param box3D        the box. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DBox3DCollision(Sphere3DReadOnly sphere3D, Box3DReadOnly box3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DBox3DCollision(sphere3D.getPosition(), box3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(box3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapesAreColliding(distance < 0.0);
   }

   private static void evaluatePoint3DBox3DCollision(Point3DReadOnly point3D, Box3DReadOnly box3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      box3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DBox3DCollision(resultToPack.getPointOnA(),
                                                                       box3D.getSize(),
                                                                       resultToPack.getPointOnB(),
                                                                       resultToPack.getNormalOnB());
      box3D.transformToWorld(resultToPack.getPointOnB());
      box3D.transformToWorld(resultToPack.getNormalOnB());
      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and a capsule.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param capsule3D    the capsule. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DCapsule3DCollision(PointShape3DReadOnly pointShape3D, Capsule3DReadOnly capsule3D,
                                                             EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(pointShape3D, capsule3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(capsule3D);
   }

   /**
    * Evaluates the collision state between two capsules.
    *
    * @param capsule3DA   the first capsule. Not modified.
    * @param capsule3DB   the second capsule. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateCapsule3DCapsule3DCollision(Capsule3DReadOnly capsule3DA, Capsule3DReadOnly capsule3DB,
                                                          EuclidShape3DCollisionResultBasics resultToPack)
   {
      Point3DBasics pointOnA = resultToPack.getPointOnA();
      Point3DBasics pointOnB = resultToPack.getPointOnB();
      double distanceBetweenAxes = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(capsule3DA.getTopCenter(),
                                                                                               capsule3DA.getBottomCenter(),
                                                                                               capsule3DB.getTopCenter(),
                                                                                               capsule3DB.getBottomCenter(),
                                                                                               pointOnA,
                                                                                               pointOnB);

      Vector3DBasics normalOnA = resultToPack.getNormalOnA();
      Vector3DBasics normalOnB = resultToPack.getNormalOnB();
      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distanceBetweenAxes);
      normalOnB.setAndNegate(normalOnA);

      pointOnA.scaleAdd(capsule3DA.getRadius(), normalOnA, pointOnA);
      pointOnB.scaleAdd(capsule3DB.getRadius(), normalOnB, pointOnB);

      double distance = distanceBetweenAxes - capsule3DA.getRadius() - capsule3DB.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);

      resultToPack.setShapeA(capsule3DA);
      resultToPack.setShapeB(capsule3DB);
   }

   /**
    * Evaluates the collision state between a sphere and a capsule.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param capsule3D    the capsule. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DCapsule3DCollision(Sphere3DReadOnly sphere3D, Capsule3DReadOnly capsule3D,
                                                         EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(sphere3D.getPosition(), capsule3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(capsule3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DCapsule3DCollision(Point3DReadOnly point3D, Capsule3DReadOnly capsule3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DCapsule3DCollision(point3D,
                                                                           capsule3D.getPosition(),
                                                                           capsule3D.getAxis(),
                                                                           capsule3D.getLength(),
                                                                           capsule3D.getRadius(),
                                                                           resultToPack.getPointOnB(),
                                                                           resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and a cylinder.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param cylinder3D   the cylinder. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DCylinder3DCollision(PointShape3DReadOnly pointShape3D, Cylinder3DReadOnly cylinder3D,
                                                              EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(pointShape3D, cylinder3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(cylinder3D);
   }

   /**
    * Evaluates the collision state between a sphere and a cylinder.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param cylinder3D   the cylinder. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DCylinder3DCollision(Sphere3DReadOnly sphere3D, Cylinder3DReadOnly cylinder3D,
                                                          EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(sphere3D.getPosition(), cylinder3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(cylinder3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DCylinder3DCollision(Point3DReadOnly point3D, Cylinder3DReadOnly cylinder3D,
                                                          EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DCylinder3DCollision(point3D,
                                                                            cylinder3D.getPosition(),
                                                                            cylinder3D.getAxis(),
                                                                            cylinder3D.getLength(),
                                                                            cylinder3D.getRadius(),
                                                                            resultToPack.getPointOnB(),
                                                                            resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and an ellipsoid.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param ellipsoid3D  the ellipsoid. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DEllipsoid3DCollision(PointShape3DReadOnly pointShape3D, Ellipsoid3DReadOnly ellipsoid3D,
                                                               EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(pointShape3D, ellipsoid3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ellipsoid3D);
   }

   /**
    * Evaluates the collision state between a point shape and an ellipsoid.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param ellipsoid3D  the ellipsoid. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DEllipsoid3DCollision(Sphere3DReadOnly sphere3D, Ellipsoid3DReadOnly ellipsoid3D,
                                                           EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(sphere3D.getPosition(), ellipsoid3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ellipsoid3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DEllipsoid3DCollision(Point3DReadOnly point3D, Ellipsoid3DReadOnly ellipsoid3D,
                                                           EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      ellipsoid3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DEllipsoid3DCollision(resultToPack.getPointOnA(),
                                                                             ellipsoid3D.getRadii(),
                                                                             resultToPack.getPointOnB(),
                                                                             resultToPack.getNormalOnB());
      ellipsoid3D.transformToWorld(resultToPack.getPointOnB());
      ellipsoid3D.transformToWorld(resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between two point shapes.
    * <p>
    * Note that the two shapes never collide.
    * </p>
    * 
    * @param shapeA       the first point shape. Not modified.
    * @param shapeB       the second point shape. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DPointShape3DCollision(PointShape3DReadOnly shapeA, PointShape3DReadOnly shapeB,
                                                                EuclidShape3DCollisionResultBasics resultToPack)
   {
      Point3DBasics pointOnA = resultToPack.getPointOnA();
      Point3DBasics pointOnB = resultToPack.getPointOnB();
      Vector3DBasics normalOnA = resultToPack.getNormalOnA();
      Vector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.set(shapeA);
      pointOnB.set(shapeB);
      double distance = pointOnA.distance(pointOnB);
      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distance);
      normalOnB.setAndNegate(normalOnA);

      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setShapesAreColliding(false);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and a ramp.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param ramp3D       the ramp. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DRamp3DCollision(PointShape3DReadOnly pointShape3D, Ramp3DReadOnly ramp3D,
                                                          EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DRamp3DCollision(pointShape3D, ramp3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ramp3D);
   }

   /**
    * Evaluates the collision state between a sphere and a ramp.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param ramp3D       the ramp. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DRamp3DCollision(Sphere3DReadOnly sphere3D, Ramp3DReadOnly ramp3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DRamp3DCollision(sphere3D.getPosition(), ramp3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ramp3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DRamp3DCollision(Point3DReadOnly point3D, Ramp3DReadOnly ramp3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      ramp3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(resultToPack.getPointOnA(),
                                                                        ramp3D.getSize(),
                                                                        resultToPack.getPointOnB(),
                                                                        resultToPack.getNormalOnB());
      ramp3D.transformToWorld(resultToPack.getPointOnB());
      ramp3D.transformToWorld(resultToPack.getNormalOnB());
      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and a sphere.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param sphere3D     the sphere. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DSphere3DCollision(PointShape3DReadOnly pointShape3D, Sphere3DReadOnly sphere3D,
                                                            EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DSphere3DCollision(pointShape3D, sphere3D.getPosition(), sphere3D.getRadius(), resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(sphere3D);
   }

   /**
    * Evaluates the collision state between two spheres.
    *
    * @param sphere3DA    the first sphere. Not modified.
    * @param sphere3DB    the second sphere. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DSphere3DCollision(Sphere3DReadOnly sphere3DA, Sphere3DReadOnly sphere3DB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DSphere3DCollision(sphere3DA.getPosition(), sphere3DB.getPosition(), sphere3DB.getRadius(), resultToPack);
      resultToPack.setShapeA(sphere3DA);
      resultToPack.setShapeB(sphere3DB);

      resultToPack.getPointOnA().scaleAdd(sphere3DA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance();
      distance -= sphere3DA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DSphere3DCollision(Point3DReadOnly point3D, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                        EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(point3D,
                                                                          sphere3DPosition,
                                                                          sphere3DRadius,
                                                                          resultToPack.getPointOnB(),
                                                                          resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   /**
    * Evaluates the collision state between a point shape and a torus.
    *
    * @param pointShape3D the point shape. Not modified.
    * @param torus3D      the torus. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluatePointShape3DTorus3DCollision(PointShape3DReadOnly pointShape3D, Torus3DReadOnly torus3D,
                                                           EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DTorus3DCollision(pointShape3D, torus3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(torus3D);
   }

   /**
    * Evaluates the collision state between a point shape and a torus.
    *
    * @param sphere3D     the sphere. Not modified.
    * @param torus3D      the torus. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    */
   public static void evaluateSphere3DTorus3DCollision(Sphere3DReadOnly sphere3D, Torus3DReadOnly torus3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DTorus3DCollision(sphere3D.getPosition(), torus3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(torus3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DTorus3DCollision(Point3DReadOnly point3D, Torus3DReadOnly torus3D, EuclidShape3DCollisionResultBasics resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DTorus3DCollision(point3D,
                                                                         torus3D.getPosition(),
                                                                         torus3D.getAxis(),
                                                                         torus3D.getRadius(),
                                                                         torus3D.getTubeRadius(),
                                                                         resultToPack.getPointOnB(),
                                                                         resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }
}
