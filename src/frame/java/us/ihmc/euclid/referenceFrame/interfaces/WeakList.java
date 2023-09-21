package us.ihmc.euclid.referenceFrame.interfaces;

import java.lang.ref.WeakReference;
import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Objects;

public class WeakList<E> extends AbstractList<E> //implements List<E>, RandomAccess
{
   private final ArrayList<WeakReference<E>> elements;
   private ElementGarbageCollectedListener listener;

   public WeakList()
   {
      elements = new ArrayList<>();

   }

   public ElementGarbageCollectedListener setGCListener(ElementGarbageCollectedListener listener)
   {
      ElementGarbageCollectedListener oldListener = this.listener;
      this.listener = listener;
      return oldListener;
   }

   public ElementGarbageCollectedListener removeGCListener()
   {
      return setGCListener(null);
   }

   @Override
   public int size()
   {
      return elements.size();
   }

   public void cleanupReferences()
   {
      for (int i = elements.size() - 1; i >= 0; i--)
      {
         if (elements.get(i).get() == null)
         {
            removeGCedElement(i);
         }
      }
   }

   private void removeGCedElement(int i)
   {
      elements.remove(i);
      if (listener != null)
         listener.onCleanup(i);
   }

   @Override
   public E get(int index)
   {
      E element = elements.get(index).get();
      while (element == null)
      {
         removeGCedElement(index);
         element = elements.get(index).get();
      }
      return element;
   }

   @Override
   public E set(int index, E element)
   {
      Objects.requireNonNull(element);
      WeakReference<E> elementReplaced = elements.set(index, new WeakReference<>(element));
      return elementReplaced == null ? null : elementReplaced.get();
   }

   @Override
   public void add(int index, E element)
   {
      Objects.requireNonNull(element);
      elements.add(index, new WeakReference<>(element));
   }

   @Override
   public E remove(int index)
   {
      WeakReference<E> elementRemoved = elements.remove(index);
      return elementRemoved == null ? null : elementRemoved.get();
   }

   @Override
   public int indexOf(Object query)
   {
      Objects.requireNonNull(query);
      for (int i = 0; i < elements.size();)
      {
         E candidate = elements.get(i).get();

         if (candidate == null)
            removeGCedElement(i);
         else if (query.equals(candidate))
            return i;
         else
            i++;
      }
      return -1;
   }

   @Override
   public int lastIndexOf(Object query)
   {
      Objects.requireNonNull(query);
      cleanupReferences();
      for (int i = elements.size() - 1; i >= 0; i--)
      {
         E candidate = elements.get(i).get();

         if (candidate == null)
            removeGCedElement(i);
         else if (query.equals(candidate))
            return i;
      }
      return -1;
   }

   @Override
   public void clear()
   {
      elements.clear();
   }

   @Override
   public boolean equals(Object o)
   {
      cleanupReferences();
      return super.equals(o);
   }

   @Override
   public int hashCode()
   {
      int hashCode = 1;
      for (int i = 0; i < size(); i++)
      {
         E e = get(i);
         hashCode = 31 * hashCode + (e == null ? 0 : e.hashCode());
      }
      return hashCode;
   }

   public static interface ElementGarbageCollectedListener
   {
      void onCleanup(int itemCollectedIndex);
   }
}
