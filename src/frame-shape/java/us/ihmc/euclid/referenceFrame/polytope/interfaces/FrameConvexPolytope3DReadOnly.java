package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameConvexPolytope3DReadOnly extends ConvexPolytope3DReadOnly, FrameShape3DReadOnly
{
   /**
    * Gets this polytope's faces.
    *
    * @return this polytope's faces.
    */
   @Override
   List<? extends FrameFace3DReadOnly> getFaces();

   /**
    * Gets the i<sup>th</sup> face of this polytope.
    *
    * @param index the face index &in; [0; {@link #getNumberOfFaces()}[.
    * @return the read-only reference to the face.
    */
   @Override
   default FrameFace3DReadOnly getFace(int index)
   {
      return getFaces().get(index);
   }

   /**
    * Gets this polytope's half-edges.
    * <p>
    * Note that the number of half-edges is twice the number of edges in this polytope.
    * </p>
    *
    * @return this polytope's half-edges.
    */
   @Override
   List<? extends FrameHalfEdge3DReadOnly> getHalfEdges();

   /**
    * Gets the i<sup>th</sup> half-edge of this polytope.
    *
    * @param index the half-edge index &in; [0; {@link #getNumberOfHalfEdges()}[.
    * @return the read-only reference to the half-edge.
    */
   @Override
   default FrameHalfEdge3DReadOnly getHalfEdge(int index)
   {
      return getHalfEdges().get(index);
   }

   /**
    * Gets this polytope's vertices.
    *
    * @return this polytope's vertices.
    */
   @Override
   List<? extends FrameVertex3DReadOnly> getVertices();

   /**
    * Gets the i<sup>th</sup> vertex of this polytope.
    *
    * @param index the vertex index &in; [0; {@link #getNumberOfVertices()}[.
    * @return the read-only reference to the vertex.
    */
   @Override
   default FrameVertex3DReadOnly getVertex(int index)
   {
      return getVertices().get(index);
   }

   /** {@inheritDoc} */
   @Override
   FrameBoundingBox3DReadOnly getBoundingBox();

   /**
    * Gets the read-only reference to the centroid of this polytope.
    *
    * @return this polytope centroid location.
    */
   @Override
   FramePoint3DReadOnly getCentroid();

   /**
    * Finds and returns the closest face to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest face to the query.
    */
   @Override
   default FrameFace3DReadOnly getClosestFace(Point3DReadOnly query)
   {
      return (FrameFace3DReadOnly) ConvexPolytope3DReadOnly.super.getClosestFace(query);
   }

   /**
    * Finds and returns the closest face to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest face to the query.
    */
   default FrameFace3DReadOnly getClosestFace(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return (FrameFace3DReadOnly) ConvexPolytope3DReadOnly.super.getClosestFace(query);
   }

   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return getSupportingVertex((FrameVertex3DReadOnly) null, supportDirection);
   }

   @Override
   default FrameVertex3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vertex3DReadOnly seed, Vector3DReadOnly supportDirection)
   {
      return (FrameVertex3DReadOnly) ConvexPolytope3DReadOnly.super.getSupportingVertex(seed, supportDirection);
   }

   default FrameVertex3DReadOnly getSupportingVertex(Vertex3DReadOnly seed, FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex(seed, (Vector3DReadOnly) supportDirection);
   }

   default FrameVertex3DReadOnly getSupportingVertex(FrameVertex3DReadOnly seed, Vector3DReadOnly supportDirection)
   {
      return getSupportingVertex((Vertex3DReadOnly) seed, supportDirection);
   }

   default FrameVertex3DReadOnly getSupportingVertex(FrameVertex3DReadOnly seed, FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vertex3DReadOnly) seed, (Vector3DReadOnly) supportDirection);
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      ConvexPolytope3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxConvexPolytope3D(this, destinationFrame, boundingBoxToPack);
   }

   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   @Override
   FrameConvexPolytope3DReadOnly copy();

   default boolean epsilonEquals(FrameConvexPolytope3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return ConvexPolytope3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameConvexPolytope3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return ConvexPolytope3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameConvexPolytope3DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return ConvexPolytope3DReadOnly.super.equals(other);
   }
}
