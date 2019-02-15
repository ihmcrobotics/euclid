package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PointShape3DReadOnly extends Shape3DReadOnly, Point3DReadOnly
{
   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   @Override
   default double getX()
   {
      return getPose().getTranslationX();
   }

   @Override
   default double getY()
   {
      return getPose().getTranslationY();
   }

   @Override
   default double getZ()
   {
      return getPose().getTranslationZ();
   }

   @Override
   default double distance(Point3DReadOnly point)
   {
      return Point3DReadOnly.super.distance(point);
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return distance(point);
   }

   @Override
   default boolean containsNaN()
   {
      return getPose().containsNaN();
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      if (closestPointOnSurfaceToPack != null)
         closestPointOnSurfaceToPack.set(this);
      return false;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(this);
      return true;
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      projectionToPack.set(this);
      return true;
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return distanceSquared(query) <= epsilon * epsilon;
   }

   default boolean epsilonEquals(PointShape3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(PointShape3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(getPose());
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates to
    * being expressed in world.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }
}
