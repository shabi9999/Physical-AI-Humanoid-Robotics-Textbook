import type {ReactNode} from 'react';

type StatItemProps = {
  icon: string;
  number: string;
  label: string;
};

function StatItem({icon, number, label}: StatItemProps): ReactNode {
  return (
    <div className="text-center">
      <div className="text-4xl mb-3">{icon}</div>
      <div className="text-5xl font-bold mb-2" style={{color: '#16a34a'}}>
        {number}
      </div>
      <div className="text-sm font-semibold uppercase tracking-widest" style={{color: '#6b7280'}}>
        {label}
      </div>
    </div>
  );
}

export default function Stats(): ReactNode {
  const stats = [
    {
      icon: '📚',
      number: '4',
      label: 'Modules'
    },
    {
      icon: '📖',
      number: '13+',
      label: 'Chapters'
    },
    {
      icon: '🎯',
      number: '50+',
      label: 'Topics'
    },
    {
      icon: '📅',
      number: '13',
      label: 'Weeks'
    }
  ];

  return (
    <section className="bg-white border-b border-gray-200 py-12 md:py-16">
      <div className="container max-w-6xl mx-auto px-6 md:px-12">
        <div className="grid grid-cols-2 md:grid-cols-4 gap-8 md:gap-12">
          {stats.map((stat, idx) => (
            <StatItem key={idx} {...stat} />
          ))}
        </div>
      </div>
    </section>
  );
}
