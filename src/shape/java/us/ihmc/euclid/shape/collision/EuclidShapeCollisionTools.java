package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EuclidShapeCollisionTools
{
   private EuclidShapeCollisionTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   public static void doPointShape3DBox3DCollisionTest(PointShape3DReadOnly pointShape3D, Box3DReadOnly box3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DBox3DCollisionTest(pointShape3D, box3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(box3D);
   }

   public static void doSphere3DBox3DCollisionTest(Sphere3DReadOnly sphere3D, Box3DReadOnly box3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DBox3DCollisionTest(sphere3D.getPosition(), box3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(box3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setDistance(distance);
      resultToPack.setShapesAreColliding(distance < 0.0);
   }

   private static void doPoint3DBox3DCollisionTest(Point3DReadOnly point3D, Box3DReadOnly box3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      box3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DBox3DCollision(resultToPack.getPointOnA(), box3D.getSize(), resultToPack.getPointOnB(),
                                                                     resultToPack.getNormalOnB());
      box3D.transformToWorld(resultToPack.getPointOnB());
      box3D.transformToWorld(resultToPack.getNormalOnB());
      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DCapsule3DCollisionTest(PointShape3DReadOnly pointShape3D, Capsule3DReadOnly capsule3D,
                                                           EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DCapsule3DCollisionTest(pointShape3D, capsule3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(capsule3D);
   }

   public static void doCapsule3DCapsule3DCollisionTest(Capsule3DReadOnly capsule3DA, Capsule3DReadOnly capsule3DB, EuclidShape3DCollisionResult resultToPack)
   {
      Point3D pointOnA = resultToPack.getPointOnA();
      Point3D pointOnB = resultToPack.getPointOnB();
      double distanceBetweenAxes = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(capsule3DA.getTopCenter(), capsule3DA.getBottomCenter(),
                                                                                               capsule3DB.getTopCenter(), capsule3DB.getBottomCenter(),
                                                                                               pointOnA, pointOnB);

      Vector3D normalOnA = resultToPack.getNormalOnA();
      Vector3D normalOnB = resultToPack.getNormalOnB();
      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distanceBetweenAxes);
      normalOnB.setAndNegate(normalOnA);

      pointOnA.scaleAdd(capsule3DA.getRadius(), normalOnA, pointOnA);
      pointOnB.scaleAdd(capsule3DB.getRadius(), normalOnB, pointOnB);

      double distance = distanceBetweenAxes - capsule3DA.getRadius() - capsule3DB.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);

      resultToPack.setShapeA(capsule3DA);
      resultToPack.setShapeB(capsule3DB);
   }

   public static void doSphere3DCapsule3DCollisionTest(Sphere3DReadOnly sphere3D, Capsule3DReadOnly capsule3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DCapsule3DCollisionTest(sphere3D.getPosition(), capsule3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(capsule3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DCapsule3DCollisionTest(Point3DReadOnly point3D, Capsule3DReadOnly capsule3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DCapsule3DCollision(point3D, capsule3D.getPosition(), capsule3D.getAxis(), capsule3D.getLength(),
                                                                         capsule3D.getRadius(), resultToPack.getPointOnB(), resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DCylinder3DCollisionTest(PointShape3DReadOnly pointShape3D, Cylinder3DReadOnly cylinder3D,
                                                            EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DCylinder3DCollisionTest(pointShape3D, cylinder3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(cylinder3D);
   }

   public static void doSphere3DCylinder3DCollisionTest(Sphere3DReadOnly sphere3D, Cylinder3DReadOnly cylinder3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DCylinder3DCollisionTest(sphere3D.getPosition(), cylinder3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(cylinder3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DCylinder3DCollisionTest(Point3DReadOnly point3D, Cylinder3DReadOnly cylinder3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DCylinder3DCollision(point3D, cylinder3D.getPosition(), cylinder3D.getAxis(), cylinder3D.getLength(),
                                                                          cylinder3D.getRadius(), resultToPack.getPointOnB(), resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DEllipsoid3DCollisionTest(PointShape3DReadOnly pointShape3D, Ellipsoid3DReadOnly ellipsoid3D,
                                                             EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ellipsoid3D);
   }

   public static void doSphere3DEllipsoid3DCollisionTest(Sphere3DReadOnly sphere3D, Ellipsoid3DReadOnly ellipsoid3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DEllipsoid3DCollisionTest(sphere3D.getPosition(), ellipsoid3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ellipsoid3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DEllipsoid3DCollisionTest(Point3DReadOnly point3D, Ellipsoid3DReadOnly ellipsoid3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      ellipsoid3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DEllipsoid3DCollision(resultToPack.getPointOnA(), ellipsoid3D.getRadii(), resultToPack.getPointOnB(),
                                                                           resultToPack.getNormalOnB());
      ellipsoid3D.transformToWorld(resultToPack.getPointOnB());
      ellipsoid3D.transformToWorld(resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DRamp3DCollisionTest(PointShape3DReadOnly pointShape3D, Ramp3DReadOnly ramp3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DRamp3DCollisionTest(pointShape3D, ramp3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ramp3D);
   }

   public static void doSphere3DRamp3DCollisionTest(Sphere3DReadOnly sphere3D, Ramp3DReadOnly ramp3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DRamp3DCollisionTest(sphere3D.getPosition(), ramp3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ramp3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DRamp3DCollisionTest(Point3DReadOnly point3D, Ramp3DReadOnly ramp3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      ramp3D.getPose().inverseTransform(point3D, resultToPack.getPointOnA());
      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(resultToPack.getPointOnA(), ramp3D.getSize(), resultToPack.getPointOnB(),
                                                                      resultToPack.getNormalOnB());
      ramp3D.transformToWorld(resultToPack.getPointOnB());
      ramp3D.transformToWorld(resultToPack.getNormalOnB());
      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DSphere3DCollisionTest(PointShape3DReadOnly pointShape3D, Sphere3DReadOnly sphere3D,
                                                          EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DSphere3DCollisionTest(pointShape3D, sphere3D.getPosition(), sphere3D.getRadius(), resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(sphere3D);
   }

   public static void doSphere3DSphere3DCollisionTest(Sphere3DReadOnly sphere3DA, Sphere3DReadOnly sphere3DB, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DSphere3DCollisionTest(sphere3DA.getPosition(), sphere3DB.getPosition(), sphere3DB.getRadius(), resultToPack);
      resultToPack.setShapeA(sphere3DA);
      resultToPack.setShapeB(sphere3DB);

      resultToPack.getPointOnA().scaleAdd(sphere3DA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance();
      distance -= sphere3DA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DSphere3DCollisionTest(Point3DReadOnly point3D, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                      EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(point3D, sphere3DPosition, sphere3DRadius, resultToPack.getPointOnB(),
                                                                        resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   public static void doPointShape3DTorus3DCollisionTest(PointShape3DReadOnly pointShape3D, Torus3DReadOnly torus3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DTorus3DCollisionTest(pointShape3D, torus3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(torus3D);
   }

   public static void doSphere3DTorus3DCollisionTest(Sphere3DReadOnly sphere3D, Torus3DReadOnly torus3D, EuclidShape3DCollisionResult resultToPack)
   {
      doPoint3DTorus3DCollisionTest(sphere3D.getPosition(), torus3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(torus3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getDistance() - sphere3D.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }

   private static void doPoint3DTorus3DCollisionTest(Point3DReadOnly point3D, Torus3DReadOnly torus3D, EuclidShape3DCollisionResult resultToPack)
   {
      resultToPack.setToNaN();
      double distance = EuclidShapeTools.evaluatePoint3DTorus3DCollision(point3D, torus3D.getPosition(), torus3D.getAxis(), torus3D.getRadius(),
                                                                       torus3D.getTubeRadius(), resultToPack.getPointOnB(), resultToPack.getNormalOnB());

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setDistance(distance);
   }
}
