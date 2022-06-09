package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a torus 3D.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Torus3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius from the torus center to the tube center.
    *
    * @return this torus main radius.
    */
   double getRadius();

   /**
    * Gets the radius of the tube of this torus.
    *
    * @return the radius of the tube.
    */
   double getTubeRadius();

   /**
    * Gets the read-only reference of the position of this position.
    *
    * @return the position of this position.
    */
   Point3DReadOnly getPosition();

   /**
    * Gets the read-only reference of this torus axis of revolution.
    *
    * @return the axis of this torus.
    */
   UnitVector3DReadOnly getAxis();

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this torus.
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
      return EuclidShapeTools.torusVolume(getRadius(), getTubeRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getAxis().containsNaN() || Double.isNaN(getRadius()) || Double.isNaN(getTubeRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.evaluatePoint3DTorus3DCollision(pointToCheck,
                                                              getPosition(),
                                                              getAxis(),
                                                              getRadius(),
                                                              getTubeRadius(),
                                                              closestPointOnSurfaceToPack,
                                                              normalAtClosestPointToPack) <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      throw new UnsupportedOperationException("Torus3D being a concave shape cannot implement the supporting vertex feature.");
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(point, getPosition(), getAxis(), getRadius(), getTubeRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideTorus3D(query, getPosition(), getAxis(), getRadius(), getTubeRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoTorus3D(pointToProject, getPosition(), getAxis(), getRadius(), getTubeRadius(), projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidShapeTools.boundingBoxCylinder3D(getPosition(), getAxis(), getTubeRadius(), getRadius() + getTubeRadius(), boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return false;
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
   Torus3DBasics copy();

   /**
    * Tests separately and on a per component basis if the pose and the radii of this torus and
    * {@code other}'s pose and radii are equal to an {@code epsilon}.
    *
    * @param object  the other object which pose and radii is to be compared against this torus pose
    *                and radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two tori are equal component-wise, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof Torus3DReadOnly))
         return false;
      Torus3DReadOnly other = (Torus3DReadOnly) object;
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getAxis().epsilonEquals(other.getAxis(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two tori are geometrically similar.
    *
    * @param object  the object to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two tori represent the same geometry, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof Torus3DReadOnly))
         return false;
      Torus3DReadOnly other = (Torus3DReadOnly) object;
      if (!EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            || !EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon) || !getPosition().geometricallyEquals(getPosition(), epsilon))
         return false;

      return EuclidGeometryTools.areVector3DsParallel(getAxis(), other.getAxis(), epsilon);
   }

   /**
    * Tests on a per component basis, if this torus 3D is exactly equal to {@code other}.
    *
    * @param other the other torus 3D to compare against this. Not modified.
    * @return {@code true} if the two tori are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Torus3DReadOnly other)
   {
      if (other == this)
      {
         return true;
      }
      else if (other == null)
      {
         return false;
      }
      else
      {
         if ((getRadius() != other.getRadius()) || (getTubeRadius() != other.getTubeRadius()) || !getPosition().equals(other.getPosition())
               || !getAxis().equals(other.getAxis()))
            return false;
         return true;
      }
   }

   /**
    * Gets a representative {@code String} of {@code torus3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.170, tube radius:  0.906]
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getTorus3DString(format, this);
   }
}
