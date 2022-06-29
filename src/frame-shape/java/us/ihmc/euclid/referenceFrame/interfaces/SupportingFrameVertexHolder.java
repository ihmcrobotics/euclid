package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.collision.epa.FrameExpandingPolytopeAlgorithm;
import us.ihmc.euclid.referenceFrame.collision.gjk.FrameGilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Any geometry implementing {@code FrameSupportingVertexHolder} can then be used with the collision
 * detectors {@link FrameGilbertJohnsonKeerthiCollisionDetector} and
 * {@link FrameExpandingPolytopeAlgorithm}.
 * <p>
 * A geometry implementing this interface needs to be able to provide the point or vertex on the
 * shape that is the farthest along a given direction. Such a point or vertex is referred to as a
 * supporting vertex.
 * </p>
 * <p>
 * Even if this interface can be implemented by concave geometries, only convex ones can be used by
 * the collision detectors.
 * </p>
 *
 * @author Sylvain Bertrand
 * @see FrameGilbertJohnsonKeerthiCollisionDetector
 * @see FrameExpandingPolytopeAlgorithm
 */
public interface SupportingFrameVertexHolder extends SupportingVertexHolder, EuclidFrameGeometry
{
   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      FramePoint3D supportingVertex = new FramePoint3D();
      boolean success = getSupportingVertex(supportDirection, supportingVertex);
      if (success)
         return supportingVertex;
      else
         return null;
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code null}. It is usually
    * the case for an empty {@code ConvexPolytope3DReadOnly}.
    * </p>
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @param supportDirection the direction to search for the farthest point on this shape. Not
    *                         modified.
    * @return the coordinates of the supporting vertex or {@code null} if this shape has currently no
    *         supporting vertex.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FramePoint3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    *
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, FixedFramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportingVertexToPack);
      return getSupportingVertex(supportDirection, (Point3DBasics) supportingVertexToPack);
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    *
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    */
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, FramePoint3DBasics supportingVertexToPack)
   {
      boolean success = getSupportingVertex(supportDirection, (Point3DBasics) supportingVertexToPack);
      if (success)
         supportingVertexToPack.setReferenceFrame(getReferenceFrame());
      return success;
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    *
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    *
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, FixedFramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }

   /**
    * Finds the supporting vertex in the given direction.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    * <p>
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    *
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * @throws ReferenceFrameMismatchException if {@code supportDirection} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean getSupportingVertex(FrameVector3DReadOnly supportDirection, FramePoint3DBasics supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection, supportingVertexToPack);
   }
}
