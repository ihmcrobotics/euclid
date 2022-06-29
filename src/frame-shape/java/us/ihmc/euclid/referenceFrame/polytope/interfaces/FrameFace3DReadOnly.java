package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a face 3D that belongs to a convex polytope 3D expressed in a given
 * reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameFace3DReadOnly extends Face3DReadOnly, SupportingFrameVertexHolder
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getCentroid();

   /** {@inheritDoc} */
   @Override
   FrameVector3DReadOnly getNormal();

   /** {@inheritDoc} */
   @Override
   FrameBoundingBox3DReadOnly getBoundingBox();

   /** {@inheritDoc} */
   @Override
   List<? extends FrameHalfEdge3DReadOnly> getEdges();

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly getEdge(int index)
   {
      return getEdges().get(index);
   }

   /** {@inheritDoc} */
   @Override
   default List<? extends FrameVertex3DReadOnly> getVertices()
   {
      return getEdges().stream().map(FrameHalfEdge3DReadOnly::getOrigin).collect(Collectors.toList());
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getVertex(int index)
   {
      return getEdge(index).getOrigin();
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer)
   {
      return (List<? extends FrameHalfEdge3DReadOnly>) Face3DReadOnly.super.lineOfSight(observer);
   }

   /**
    * From the point of view of an observer located outside the face, only a continuous subset of the
    * face's edges can be seen defining a line-of-sight. This method finds the edges in order that are
    * in the line-of-sight.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the edges in order that are in the line-of-sight.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSight((Point3DReadOnly) observer);
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(Point3DReadOnly observer, double epsilon)
   {
      return (List<? extends FrameHalfEdge3DReadOnly>) Face3DReadOnly.super.lineOfSight(observer, epsilon);
   }

   /**
    * Collects and returns the edges that are in the line-of-sight of the given observer in a similar
    * way as {@link #lineOfSight(Point3DReadOnly)}.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    * <p>
    * The line-of-sight is possibly extended to include the edge right before and right after if the
    * observer is within {@code epsilon} of the their support lines.
    * </p>
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param epsilon  tolerance to determine whether to extend the line-of-sight.
    * @return the edges in order that are in the line-of-sight.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default List<? extends FrameHalfEdge3DReadOnly> lineOfSight(FramePoint3DReadOnly observer, double epsilon)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSight((Point3DReadOnly) observer, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly lineOfSightStart(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.lineOfSightStart(observer);
   }

   /**
    * Finds and returns the first edge that is visible from the observer.
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the first visible half edge.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FrameHalfEdge3DReadOnly lineOfSightStart(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightStart((Point3DReadOnly) observer);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly lineOfSightEnd(Point3DReadOnly observer)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.lineOfSightEnd(observer);
   }

   /**
    * Finds and returns the last edge that is visible from the observer.
    * <p>
    * In case the observer located directly above, below, or on the face, this method returns
    * {@code null}.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return the last visible half edge.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FrameHalfEdge3DReadOnly lineOfSightEnd(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return lineOfSightEnd((Point3DReadOnly) observer);
   }

   /**
    * Tests whether the i<sup>th</sup> edge of this face is visible from the observer.
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param index    the index &in; [0; {@link #getNumberOfEdges()}[ of the edge to test.
    * @return {@code true} if the observer can see the edge, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean canObserverSeeEdge(FramePoint3DReadOnly observer, int index)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeEdge(observer, index);
   }

   /**
    * Tests whether the given edge is visible from the observer.
    * <p>
    * It is assumed that the given edge belongs to this face.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param edge     the edge to test. Not modified.
    * @return {@code true} if the observer can see the edge, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the {@code observer} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean canObserverSeeEdge(FramePoint3DReadOnly observer, FrameHalfEdge3DReadOnly edge)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeEdge(observer, edge);
   }

   /**
    * Tests whether this face is visible from the observer.
    * <p>
    * The face is visible from an observer if the face's normal is pointing towards it.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @return {@code true} if the observer can see this face, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean canObserverSeeFace(FramePoint3DReadOnly observer)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeFace(observer);
   }

   /**
    * Tests whether this face is visible from the observer.
    * <p>
    * The face is visible from an observer if the face's normal is pointing towards it and that the
    * observer is located at a distance greater than {@code epsilon} from this face support plane.
    * </p>
    *
    * @param observer the coordinates of the observer. Not modified.
    * @param epsilon  the minimum distance between the observer and this face support plane for the
    *                 face to be visible.
    * @return {@code true} if the observer can see this face, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean canObserverSeeFace(FramePoint3DReadOnly observer, double epsilon)
   {
      checkReferenceFrameMatch(observer);
      return Face3DReadOnly.super.canObserverSeeFace(observer, epsilon);
   }

   /**
    * Tests whether the query is within {@code epsilon} of this face's support plane.
    *
    * @param query   the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be on this face's support plane, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean isPointInFaceSupportPlane(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointInFaceSupportPlane(query, epsilon);
   }

   /**
    * Tests whether the query is located inside this face.
    *
    * @param query   the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the point is an interior point, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean isPointInside(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointInside(query, epsilon);
   }

   /**
    * Tests whether the query is located directly above or below this face, such its projection would
    * be located inside this face.
    *
    * @param query the coordinates of the query. Not modified.
    * @return {@code true} if the query is located either directly above or below this face,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isPointDirectlyAboveOrBelow(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.isPointDirectlyAboveOrBelow(query);
   }

   /** {@inheritDoc} */
   @Override
   default FrameFace3DReadOnly getNeighbor(int index)
   {
      return (FrameFace3DReadOnly) Face3DReadOnly.super.getNeighbor(index);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly getCommonEdgeWith(Face3DReadOnly neighbor)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getCommonEdgeWith(neighbor);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly getClosestEdge(Point3DReadOnly query)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestEdge(query);
   }

   /**
    * Finds and returns the closest edge to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest edge to the query.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FrameHalfEdge3DReadOnly getClosestEdge(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestEdge(query);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly getClosestVisibleEdge(Point3DReadOnly query)
   {
      return (FrameHalfEdge3DReadOnly) Face3DReadOnly.super.getClosestVisibleEdge(query);
   }

   /**
    * Finds and returns the closest visible edge to the query.
    * <p>
    * If the query is located directly above, below, or on the face, this method returns {@code null}.
    * </p>
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest visible edge to the query, or {@code null} if there is no visible edge from
    *         the query.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FrameHalfEdge3DReadOnly getClosestVisibleEdge(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return getClosestVisibleEdge((Point3DReadOnly) query);
   }

   /**
    * Computes the minimum distance between a given point and the infinitely large plane supporting
    * this face.
    * <p>
    * The returned value is signed as follows: positive if the query is located above the face's
    * support plane, negative if it is located below.
    * </p>
    *
    * @param query the coordinates of the query. Not modified.
    * @return the signed minimum distance between the point and this face's support plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double signedDistanceFromSupportPlane(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.signedDistanceFromSupportPlane(query);
   }

   /**
    * Returns the minimum distance between this face and the given point.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the minimum distance between the point and this face.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double distance(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.distance(query);
   }

   /**
    * Computes the minimum distance between a given point and the infinitely large plane supporting
    * this face.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the minimum distance between the point and this face's support plane.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double distanceFromSupportPlane(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return Face3DReadOnly.super.distanceFromSupportPlane(query);
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this face or {@code null} if the method failed.
    */
   @Override
   default FramePoint3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      FramePoint3D projection = new FramePoint3D();
      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this face or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FramePoint3DBasics orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjectionCopy((Point3DReadOnly) pointToProject);
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this face is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code pointToProject} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return Face3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a point on this face.
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this face is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject, projectionToPack);
      return Face3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return (FrameVertex3DReadOnly) Face3DReadOnly.super.getSupportingVertex(supportDirection);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   /**
    * Gets the representative {@code String} of this frame face 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    *    worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameFace3DString(format, this);
   }
}
