package us.ihmc.euclid.geometry.interfaces;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implement this interface to create a custom supplier of 3D vertices or use the static methods to
 * create default suppliers.
 *
 * @author Sylvain Bertrand
 */
public interface Vertex3DSupplier extends EuclidGeometry
{
   /**
    * A supplier with no vertices.
    */
   static final Vertex3DSupplier EMPTY_SUPPLIER = new Vertex3DSupplier()
   {
      @Override
      public Point3DReadOnly getVertex(int index)
      {
         return null;
      }

      @Override
      public int getNumberOfVertices()
      {
         return 0;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Vertex3DSupplier)
            return equals((Vertex3DSupplier) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Gets the vertex corresponding to the given index.
    *
    * @param index the index of the vertex, {@code index} &in; [0, {@code getNumberOfVertices()}[.
    * @return the vertex.
    */
   Point3DReadOnly getVertex(int index);

   /**
    * Gets the number of vertices that this supplier holds.
    *
    * @return the number of vertices.
    */
   int getNumberOfVertices();

   /**
    * Tests whether this supplier is empty or not.
    *
    * @return {@code true} if this supplier has no vertices, {@code false} if it has at one vertex.
    */
   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Vertex3DSupplier))
         return false;
      Vertex3DSupplier other = (Vertex3DSupplier) geometry;
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).equals(other.getVertex(i)))
            return false;
      }
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Vertex3DSupplier))
         return false;

      Vertex3DSupplier other = (Vertex3DSupplier) geometry;

      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).epsilonEquals(other.getVertex(i), epsilon))
            return false;
      }

      return true;
   }

   /**
    * Tests on a per-vertex basis if this supplier and {@code other} are equal to an {@code epsilon}.
    * <p>
    * The difference with {@link #epsilonEquals(EuclidGeometry, double)} is this method relies on
    * {@link Point3DReadOnly#geometricallyEquals(EuclidGeometry, double)}.
    * </p>
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Vertex3DSupplier))
         return false;

      Vertex3DSupplier other = (Vertex3DSupplier) geometry;

      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).geometricallyEquals(other.getVertex(i), epsilon))
            return false;
      }

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default String toString(String format)
   {
      StringBuilder sb = new StringBuilder("Vertex 3D Supplier: [");

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (i > 0)
            sb.append(", ");
         sb.append(getVertex(i).toString(format));
      }

      sb.append(']');

      return sb.toString();
   }

   /**
    * Gets an empty supplier.
    *
    * @return the supplier.
    */
   static Vertex3DSupplier emptyVertex3DSupplier()
   {
      return EMPTY_SUPPLIER;
   }

   /**
    * Returns a fixed-size supplier backed by the given array.
    *
    * @param vertices the array by which the supplier will be backed.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly... vertices)
   {
      return asVertex3DSupplier(Arrays.asList(vertices));
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array starting with the first
    * element and specified by its length {@code numberOfVertices}.
    *
    * @param vertices         the array by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex3DSupplier(Arrays.asList(vertices), numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    *
    * @param vertices         the array by which the supplier will be backed.
    * @param startIndex       the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyVertex3DSupplier();
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return asVertex3DSupplier(Arrays.asList(vertices), startIndex, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by the given list.
    *
    * @param vertices the list by which the supplier will be backed.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices)
   {
      return asVertex3DSupplier(vertices, vertices.size());
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list starting with the first
    * element and specified by its length {@code numberOfVertices}.
    *
    * @param vertices         the list by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      return asVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    *
    * @param vertices         the list by which the supplier will be backed.
    * @param startIndex       the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyVertex3DSupplier();
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new Vertex3DSupplier()
      {
         @Override
         public Point3DReadOnly getVertex(int index)
         {
            return vertices.get(index + startIndex);
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }

         @Override
         public int hashCode()
         {
            long bits = 1;
            for (int i = 0; i < getNumberOfVertices(); i++)
            {
               bits = EuclidHashCodeTools.addToHashCode(bits, getVertex(i));
            }
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Vertex3DSupplier)
               return equals((Vertex3DSupplier) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }
      };
   }
}
