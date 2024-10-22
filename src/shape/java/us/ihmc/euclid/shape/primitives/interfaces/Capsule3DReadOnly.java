package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a capsule 3D.
 * <p>
 * A capsule 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Capsule3DReadOnly extends Shape3DReadOnly
{

   /**
    * Gets the length of this capsule.
    *
    * @return the length.
    */
   double getLength();

   /**
    * Gets the half-length of this capsule.
    *
    * @return the half-length.
    */
   default double getHalfLength()
   {
      return 0.5 * getLength();
   }

   /**
    * Gets the radius of this capsule.
    *
    * @return the radius.
    */
   double getRadius();

   /**
    * Gets the read-only reference of the position of this capsule center.
    *
    * @return the position of this capsule.
    */
   Point3DReadOnly getPosition();

   /**
    * Gets the read-only reference of this capsule axis of revolution.
    *
    * @return the axis of this capsule.
    */
   UnitVector3DReadOnly getAxis();

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this capsule.
    * </p>
    */
   @Override
   default Point3DReadOnly getCentroid()
   {
      return getPosition();
   }

   /** {@inheritDoc} */
   @Override
   default double getVolume()
   {
      return EuclidShapeTools.capsuleVolume(getRadius(), getLength());
   }

   /**
    * Gets the read-only reference to the center of the top half-sphere.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return the top center.
    */
   default Point3DReadOnly getTopCenter()
   {
      Point3D topCenter = new Point3D();
      topCenter.scaleAdd(getHalfLength(), getAxis(), getPosition());
      return topCenter;
   }

   /**
    * Gets the read-only reference to the center of the bottom half-sphere.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return the bottom center.
    */
   default Point3DReadOnly getBottomCenter()
   {
      Point3D bottomCenter = new Point3D();
      bottomCenter.scaleAdd(-getHalfLength(), getAxis(), getPosition());
      return bottomCenter;
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getAxis().containsNaN() || Double.isNaN(getLength()) || Double.isNaN(getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.evaluatePoint3DCapsule3DCollision(pointToCheck,
                                                                getPosition(),
                                                                getAxis(),
                                                                getLength(),
                                                                getRadius(),
                                                                closestPointOnSurfaceToPack,
                                                                normalAtClosestPointToPack) <= 0.0;
   }

   @Override
   default int intersectionWith(Point3DReadOnly pointOnLine,
                                Vector3DReadOnly lineDirection,
                                Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      throw new UnsupportedOperationException("Intersection line capsule isn't supported.");
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      EuclidShapeTools.supportingVertexCapsule3D(supportDirection, getPosition(), getAxis(), getLength(), getRadius(), supportingVertexToPack);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndCapsule3D(point, getPosition(), getAxis(), getLength(), getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideCapsule3D(query, getPosition(), getAxis(), getLength(), getRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoCapsule3D(pointToProject, getPosition(), getAxis(), getLength(), getRadius(), projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPrimitive()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isDefinedByPose()
   {
      return false;
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default Shape3DPoseReadOnly getPose()
   {
      return null;
   }

   @Override
   Capsule3DBasics copy();

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Capsule3DReadOnly))
         return false;
      Capsule3DReadOnly other = (Capsule3DReadOnly) geometry;
      if (!EuclidCoreTools.epsilonEquals(getLength(), other.getLength(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon))
         return false;
      if (!getPosition().epsilonEquals(other.getPosition(), epsilon))
         return false;
      if (!other.getAxis().epsilonEquals(other.getAxis(), epsilon))
         return false;
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Capsule3DReadOnly))
         return false;
      Capsule3DReadOnly other = (Capsule3DReadOnly) geometry;
      if (!EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getLength(), other.getLength(), epsilon))
         return false;
      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;

      return EuclidGeometryTools.areVector3DsParallel(getAxis(), other.getAxis(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Capsule3DReadOnly))
         return false;
      Capsule3DReadOnly other = (Capsule3DReadOnly) geometry;

      if (!EuclidCoreTools.equals(getRadius(), other.getRadius()))
         return false;
      if (!EuclidCoreTools.equals(getLength(), other.getLength()))
         return false;
      if (!getPosition().equals(other.getPosition()))
         return false;
      if (!getAxis().equals(other.getAxis()))
         return false;
      return true;

   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidShapeTools.boundingBoxCapsule3D(getPosition(), getAxis(), getLength(), getRadius(), boundingBoxToPack);
   }

   /**
    * Gets a representative {@code String} of this capsule 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getCapsule3DString(format, this);
   }
}
