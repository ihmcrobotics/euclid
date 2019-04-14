package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.shape.collision.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Any geometry implementing {@code SupportingVertexHolder} can then be used with the collision
 * detectors {@link GilbertJohnsonKeerthiCollisionDetector} and {@link ExpandingPolytopeAlgorithm}.
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
 */
public interface SupportingVertexHolder
{
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
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param supportDirection the direction to search for the farthest point on this shape. Not
    *           modified.
    * @return the coordinates of the supporting vertex or {@code null} if this shape has currently no
    *         supporting vertex.
    */
   default Point3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      Point3D supportingVertex = new Point3D();
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
    * Edge-case: this shape is "empty" or "void", this method then returns {@code false} and
    * {@code supportingVertexToPack} remains unchanged. It is usually the case for an empty
    * {@code ConvexPolytope3DReadOnly}.
    * </p>
    * 
    * @param supportDirection the direction to search for the farthest point on this shape. Not
    *           modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * 
    */
   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);
}
