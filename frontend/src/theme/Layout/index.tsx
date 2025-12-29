import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@docusaurus/theme-common/lib/internal';
import type {WrapperProps} from '@docusaurus/types';
import CustomChatbot from '../../components/CustomChatbot'; // Import CustomChatbot

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <>
      <Layout {...props} />
      <CustomChatbot /> {/* Render the CustomChatbot here */}
    </>
  );
}
