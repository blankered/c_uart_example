   M S B A M L   `   `   `  ����
  NatCap    NatCap.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  NatCap��5   	   $	��600��6   $	��800��6%   ���	#FF5F5F5F���	#FF333333 � 
ResizeMode$ CanResizeWithGrip=�6`    � WindowStyle$ SingleBorderWindow=�6   ���	#FFD1D1D1 � 	Resources 5         MyContextMenu m�Name��%v   &	        �� 5   
   .@�5      m� $ clearBoxMenuItem��6   -   5        $��Clear��5   2   6A   5	      5
      .��5      �  � RowDefinitions 5   
   �� 5      $��60 �6   �� 5      $��* �6   .+�5   
   �  � ColumnDefinitions 5      �� 5      $	��230 �6#   �� 5      $��75 �6#   �� 5      $��* �6#   �� 5      $	��150 �6#   .+�5      �� $��0��6    �� Orientation$ 
Horizontal=�6*   .+�5      d� -   5        $��buttonStart��5      $��Record��63   $��10q�6D   $��33��6P   $
��Left=�6\   $	��Top=�6w   $��80��6�    d� 	IsChecked$ FalseY�6�   �� 5      -   5        $��
buttonStop��5      $
��Stop��6+   $��10q�6:   $��33��6F   $
��Left=�6R   $	��Top=�6m   $��80��6�   6�   5      �� 5      -   5        $��
textBlock1��5      $��1��6*   $��23��6:   $��	Take Name��6F   $��
10,10,0,10q�6W   $��Center=�6k   $
��Left=�6�   �� 5      -   5        $��textBox1��5      $��2��6&   $��23��67   $��Stretch=�6C   $��10,10,10,10q�6a   $��Center=�6w   �� 5      -   5        $��comboBoxLocal��5      $��3��6,   $��23��6<   $	��140��6H   6T   5       � 5"   
   $��1��6    5#      �� 5$      �� 5%      $
��Auto �6#   �� 5&      .+�5(      �� -   5        $��textBox2��5(      $��0��6&   $��Stretch=�66   $��Stretch=�6T   "	 �� VerticalScrollBarVisibility$
	 Auto=�6p   
 �� 
IsReadOnly
 .  � ContextMenu# [  6�   �� 5)      $��1��6   $��Center=�6+   $��Stretch=�6H    �� ShowsPreview . 6d   �� 5*      -	   5        $��listBox1��5*      $��2��6&   # [  66   5+      5-      5/      