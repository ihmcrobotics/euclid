package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.List;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Vertex3DReadOnly extends Point3DReadOnly, Simplex3D
{
   /**
    * Get list of edges that originate at this vertex
    * 
    * @return a list of read only references to the edges that originate at this edge
    */
   List<? extends HalfEdge3DReadOnly> getAssociatedEdges();

   /**
    * Get a particular associated edge based on its index in the associated edge list held by the
    * vertex.
    * 
    * @param index must be less than value returned by {@code getNumberOfAssociatedEdges()}
    * @return a read only reference to the edge
    */
   HalfEdge3DReadOnly getAssociatedEdge(int index);

   /**
    * Checks if the edge specified originates at the current vertex. Check is performed by comparing
    * objects and not geometrical closeness
    * 
    * @param the half edge that is to be checked for association
    * @return {@code true} if the specified edge is on the associated edge list, {@code false}
    *         otherwise
    */
   default boolean isAssociatedWithEdge(HalfEdge3DReadOnly edgeToCheck)
   {
      return getAssociatedEdges().contains(edgeToCheck);
   }

   /**
    * An associated edge is sufficiently close if its origin and destination are within a
    * {@code epsilon} distance of the corresponding points for the specified edge
    * 
    * @param edgeToCheck
    * @param epsilon
    * @return {@code true} if the specified edge is geometrically close to any of the associated edges.
    *         Otherwise {@code false}
    */
   default boolean isAssociatedWithEdge(HalfEdge3DReadOnly edgeToCheck, double epsilon)
   {
      for (int i = 0; i < getAssociatedEdges().size(); i++)
      {
         if (getAssociatedEdges().get(i).epsilonEquals(edgeToCheck, epsilon))
            return true;
      }
      return false;
   }

   /**
    * Returns the number of references held in the associated edge list
    * 
    * @return integer number of edges that originate at this vertex
    */
   int getNumberOfAssociatedEdges();

   /**
    * Calculates the dot product of the specified vector and the vector from the origin to this vertex
    * 
    * @return the resultant dot product
    */
   default double dot(Vector3DReadOnly vector)
   {
      return getX() * vector.getX() + getY() * vector.getY() + getZ() * vector.getZ();
   }

   @Override
   default double distance(Point3DReadOnly other)
   {
      return Point3DReadOnly.super.distance(other);
   }

   @Override
   default void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      supportVectorToPack.sub(point, this);
   }

   @Override
   default Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return this;
   }

   default boolean equals(Vertex3DReadOnly other)
   {
      return Point3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(Vertex3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(Vertex3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Get a printable version of the vertex data
    * 
    * @return string indicating the spatial coordinates for this vertex
    */
   String toString();

   /**
    * Checks if any of the associated faces for this vertex have been marked.
    * <p>
    * Marking is simply a boolean value being set to {@code true} or {@code false} Useful for some
    * operations that need a temporary list to be stored but you're lazy and don't want to make that
    * list and look it up all the time
    * </p>
    * 
    * @return {@code true} if the face is not null and marked. Otherwise {@code false}
    */
   boolean isAnyFaceMarked();
}