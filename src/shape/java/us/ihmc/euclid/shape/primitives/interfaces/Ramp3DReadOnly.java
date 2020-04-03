package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a ramp 3D.
 * <p>
 * A ramp represents a 3D shape with a triangular section in the XZ-plane. Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Ramp3DReadOnly extends Shape3DReadOnly
{
   /**
    * Get the read-only reference to the size along the three local axes of this ramp.
    *
    * @return the size of this ramp.
    */
   Vector3DReadOnly getSize();

   /**
    * Gets the read-only reference to the pose of this ramp.
    *
    * @return the pose of this ramp.
    */
   @Override
   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this ramp.
    *
    * @return the orientation of this ramp.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this ramp.
    *
    * @return the position of this ramp.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * Gets the intermediate variable supplier that can be used for performing operations in either a
    * garbage-free of thread-safe manner.
    *
    * @return the intermediate variable supplier.
    */
   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPose().containsNaN() || getSize().containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics pointToCheckInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, pointToCheckInLocal);

      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(pointToCheckInLocal,
                                                                        getSize(),
                                                                        closestPointOnSurfaceToPack,
                                                                        normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToCheckInLocal);

      return distance <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (getOrientation().isIdentity())
      {
         EuclidShapeTools.supportingVectexRamp3D(supportDirection, getSize(), supportingVertexToPack);
         supportingVertexToPack.add(getPosition());
      }
      else
      {
         Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
         getPose().inverseTransform(supportDirection, supportDirectionInLocal);

         EuclidShapeTools.supportingVectexRamp3D(supportDirectionInLocal, getSize(), supportingVertexToPack);
         transformToWorld(supportingVertexToPack);

         getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);
      }

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, pointInLocal);

      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndRamp3D(pointInLocal, getSize());

      getIntermediateVariableSupplier().releasePoint3D(pointInLocal);

      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);

      boolean isInside = EuclidShapeTools.isPoint3DInsideRamp3D(queryInLocal, getSize(), epsilon);

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointToProjectInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointToProjectInLocal);

      boolean hasBeenProjected = EuclidShapeTools.orthogonalProjectionOntoRamp3D(pointToProjectInLocal, getSize(), projectionToPack);

      if (hasBeenProjected)
         transformToWorld(projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToProjectInLocal);

      return hasBeenProjected;
   }

   /**
    * Gets the size of this ramp along the x-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets the size of this ramp along the y-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets the size of this ramp along the z-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   default double getRampLength()
   {
      return EuclidShapeTools.computeRamp3DLength(getSize());
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   default double getRampIncline()
   {
      return EuclidShapeTools.computeRamp3DIncline(getSize());
   }

   /**
    * Computes and returns the surface normal of the slope face of this ramp.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return the surface normal of the slope.
    */
   default Vector3DBasics getRampSurfaceNormal()
   {
      Vector3D surfaceNormal = new Vector3D();
      getRampSurfaceNormal(surfaceNormal);
      return surfaceNormal;
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   default void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-getSizeZ() / getRampLength(), 0.0, getSizeX() / getRampLength());
      transformToWorld(surfaceNormalToPack);
   }

   /**
    * Gets the 6 vertices, expressed in world, of this ramp as an array.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return an array of 6 {@code Point3D} with this ramp vertices.
    */
   default Point3DBasics[] getVertices()
   {
      Point3D[] vertices = new Point3D[6];
      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 6 vertices of this ramp in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 6.
    * @throws NullPointerException     if any of the 6 first elements of the given array is
    *                                  {@code null}.
    */
   default void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Packs the world coordinates of one of this ramp vertices.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @param vertexIndex the index in [0, 5] of the vertex to pack.
    * @return the coordinates of the vertex.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 5].
    */
   default Point3DBasics getVertex(int vertexIndex)
   {
      Point3D vertex = new Point3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   /**
    * Packs the world coordinates of one of this ramp vertices.
    *
    * @param vertexIndex  the index in [0, 5] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 5].
    */
   default void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 6)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 5], was: " + vertexIndex);

      vertexToPack.set((vertexIndex & 2) == 0 ? getSizeX() : 0.0,
                       (vertexIndex & 1) == 0 ? 0.5 * getSizeY() : -0.5 * getSizeY(),
                       (vertexIndex & 4) == 0 ? 0.0 : getSizeZ());
      transformToWorld(vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();
      Point3DBasics vertex = getIntermediateVariableSupplier().requestPoint3D();

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
      {
         getVertex(vertexIndex, vertex);
         boundingBoxToPack.updateToIncludePoint(vertex);
      }

      getIntermediateVariableSupplier().releasePoint3D(vertex);
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
      return true;
   }

   @Override
   Ramp3DBasics copy();

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other   the other ramp which pose and size is to be compared against this ramp pose and
    *                size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other   the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().geometricallyEquals(other.getPosition(), epsilon)
            && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }

   /**
    * Tests on a per component basis, if this ramp 3D is exactly equal to {@code other}.
    *
    * @param other the other ramp 3D to compare against this. Not modified.
    * @return {@code true} if the two ramps are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Ramp3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *                      Modified.
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
    *                      Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }
}
