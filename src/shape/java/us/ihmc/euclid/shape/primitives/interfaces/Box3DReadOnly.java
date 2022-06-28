package us.ihmc.euclid.shape.primitives.interfaces;

import static us.ihmc.euclid.tools.TupleTools.dot;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a box 3D.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Box3DReadOnly extends Shape3DReadOnly
{
   /**
    * Get the read-only reference to the size along the three local axes of this box.
    *
    * @return the size of this box.
    */
   Vector3DReadOnly getSize();

   /**
    * Gets the read-only reference to the pose of this box.
    * <p>
    * The position part describes the coordinates of the center.
    * </p>
    *
    * @return the pose of this box.
    */
   @Override
   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this box.
    *
    * @return the orientation of this box.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this box's center.
    *
    * @return the position of this box.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this box.
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
      return EuclidShapeTools.boxVolume(getSizeX(), getSizeY(), getSizeZ());
   }

   /**
    * Checks that the size component corresponding to the given axis is positive.
    *
    * @param axis to identify the component to check.
    * @throws IllegalArgumentException if the size component is strictly negative.
    */
   default void checkSizePositive(Axis3D axis)
   {
      if (getSize().getElement(axis) < 0.0)
         throw new IllegalArgumentException("The " + axis + "-size of a " + getClass().getSimpleName() + " cannot be negative: " + getSize().getElement(axis));
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
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, queryInLocal);

      double distance = EuclidShapeTools.evaluatePoint3DBox3DCollision(queryInLocal, getSize(), closestPointOnSurfaceToPack, normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);
      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return distance <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (getOrientation().isIdentity())
      {
         EuclidShapeTools.supportingVertexBox3D(supportDirection, getSize(), supportingVertexToPack);
         supportingVertexToPack.add(getPosition());
      }
      else
      {
         Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
         getPose().inverseTransform(supportDirection, supportDirectionInLocal);

         EuclidShapeTools.supportingVertexBox3D(supportDirectionInLocal, getSize(), supportingVertexToPack);
         transformToWorld(supportingVertexToPack);
         getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);
      }
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, queryInLocal);

      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndBox3D(queryInLocal, getSize());

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      double dX = query.getX() - getPose().getTranslationX();
      double dY = query.getY() - getPose().getTranslationY();
      double dZ = query.getZ() - getPose().getTranslationZ();

      if (getPose().getShapeOrientation().isIdentity())
         return EuclidShapeTools.isPoint3DInsideBox3D(dX, dY, dZ, getSize(), epsilon);

      double xLocalQuery = dot(dX, dY, dZ, getPose().getXAxis());

      if (Math.abs(xLocalQuery) <= 0.5 * getSizeX() + epsilon)
      {
         double yLocalQuery = dot(dX, dY, dZ, getPose().getYAxis());

         if (Math.abs(yLocalQuery) <= 0.5 * getSizeY() + epsilon)
         {
            double zLocalQuery = dot(dX, dY, dZ, getPose().getZAxis());
            return Math.abs(zLocalQuery) <= 0.5 * getSizeZ() + epsilon;
         }
      }
      return false;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointToProjectInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointToProjectInLocal);

      boolean hasBeenProjected = EuclidShapeTools.orthogonalProjectionOntoBox3D(pointToProjectInLocal, getSize(), projectionToPack);

      if (hasBeenProjected)
         transformToWorld(projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToProjectInLocal);
      return hasBeenProjected;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 box. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine,
                                Vector3DReadOnly lineDirection,
                                Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      double maxX = 0.5 * getSizeX();
      double maxY = 0.5 * getSizeY();
      double maxZ = 0.5 * getSizeZ();
      double minX = -maxX;
      double minY = -maxY;
      double minZ = -maxZ;

      Point3DBasics pointOnLineInLocal = getIntermediateVariableSupplier().requestPoint3D();
      Vector3DBasics lineDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();

      getPose().inverseTransform(pointOnLine, pointOnLineInLocal);
      getPose().inverseTransform(lineDirection, lineDirectionInLocal);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(minX,
                                                                                                minY,
                                                                                                minZ,
                                                                                                maxX,
                                                                                                maxY,
                                                                                                maxZ,
                                                                                                pointOnLineInLocal,
                                                                                                lineDirectionInLocal,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointOnLineInLocal);
      getIntermediateVariableSupplier().releaseVector3D(lineDirectionInLocal);

      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);
      return numberOfIntersections;
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

   /**
    * Gets this box size along the x-axis.
    *
    * @return this box size along the x-axis.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets this box size along the y-axis.
    *
    * @return this box size along the y-axis.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets this box size along the z-axis.
    *
    * @return this box size along the z-axis.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   /**
    * Computes and packs the bounding box 3D that fully contains this box 3D.
    * <p>
    * This method does not take the size of this box to convert it into a bounding box, it actually
    * considers the pose of this box 3D such that if it has a non-zero orientation the bounding box
    * will be bigger than this box.
    * </p>
    *
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidShapeTools.boundingBoxBox3D(getPosition(), getOrientation(), getSize(), boundingBoxToPack);
   }

   /**
    * Gets the {@code ConvexPolytope3DReadOnly} view backed this box.
    *
    * @return the polytope view of this box.
    */
   BoxPolytope3DView asConvexPolytope();

   /**
    * Gets the 8 vertices, expressed in world, of this box as an array.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return an array of 8 {@code Point3D} with this box vertices.
    */
   default Point3DBasics[] getVertices()
   {
      Point3D[] vertices = new Point3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 8 vertices of this box in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 8.
    * @throws NullPointerException     if any of the 8 first elements of the given array is
    *                                  {@code null}.
    */
   default void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Packs the world coordinates of one of this box vertices.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @param vertexIndex the index in [0, 7] of the vertex to pack.
    * @return the coordinates of the vertex.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 7].
    */
   default Point3DBasics getVertex(int vertexIndex)
   {
      Point3D vertex = new Point3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   /**
    * Packs the world coordinates of one of this box vertices.
    *
    * @param vertexIndex  the index in [0, 7] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 7].
    */
   default void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 8)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 7], was: " + vertexIndex);

      vertexToPack.set((vertexIndex & 1) == 0 ? getSizeX() : -getSizeX(),
                       (vertexIndex & 2) == 0 ? getSizeY() : -getSizeY(),
                       (vertexIndex & 4) == 0 ? getSizeZ() : -getSizeZ());
      vertexToPack.scale(0.5);
      transformToWorld(vertexToPack);
   }

   @Override
   Box3DBasics copy();

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Box3DReadOnly))
         return false;
      Box3DReadOnly other = (Box3DReadOnly) geometry;
      return getSize().epsilonEquals(other.getSize(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon)
            && getPosition().epsilonEquals(other.getPosition(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Box3DReadOnly))
         return false;
      Box3DReadOnly other = (Box3DReadOnly) geometry;
      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;

      Vector3DBasics otherSize = getIntermediateVariableSupplier().requestVector3D();
      other.getPose().transform(other.getSize(), otherSize);
      transformToLocal(otherSize);
      otherSize.absolute();

      boolean result = getSize().geometricallyEquals(otherSize, epsilon);
      getIntermediateVariableSupplier().releaseVector3D(otherSize);
      return result;
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Box3DReadOnly))
         return false;
      Box3DReadOnly other = (Box3DReadOnly) geometry;
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

   /**
    * Gets the representative {@code String} of {@code box3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getBox3DString(format, this);
   }
}
